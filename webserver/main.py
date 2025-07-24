# webserver/main.py
from fastapi import FastAPI, WebSocket, Request, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
import uvicorn
import asyncio
import json
from typing import List
from llm import get_trip_advice

# Custom Firebase client
from firebase_client import FirebaseClient

# --- Core Application Objects ---
app = FastAPI()
firebase = FirebaseClient(credential_path='serviceAccountKey.json')

# This will hold the single, active detector instance's state
# In a real-world multi-driver scenario, this would be a dictionary keyed by deviceId
active_trip_data = {
    "is_active": False,
    "start_time": None,
    "events": dict[str, int](),  # Dictionary to hold event counts
    "last_safety_score": 100,
    "last_eco_score": 100,
    "medium_safety_score": 100,
    "medium_eco_score": 100,
    "number_of_events": 1,
}

# --- WebSocket Management ---
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

manager = ConnectionManager()


# --- HTML Frontend Endpoint ---
@app.get("/")
async def get_frontend():
    with open("index.html", "r") as f:
        html = f.read()
    return HTMLResponse(html)

# --- WebSocket Endpoint ---
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            action = message.get("action")

            if action == "get_device_info":
                info = firebase.get_device_info()
                await websocket.send_json({"action": "device_info", **info})

            elif action == "start_trip":
                print("Received 'start_trip' command.")
                active_trip_data["is_active"] = True
                active_trip_data["start_time"] = asyncio.get_event_loop().time()
                active_trip_data["events"] = {}
                active_trip_data["last_safety_score"] = 100
                active_trip_data["last_eco_score"] = 100
                active_trip_data["medium_safety_score"] = 100
                active_trip_data["medium_eco_score"] = 100
                active_trip_data["number_of_events"] = 1
                await manager.broadcast(json.dumps({"action": "trip_started"}))

            elif action == "end_trip":
                print("Received 'end_trip' command.")
                active_trip_data["is_active"] = False
                
                # Finalize trip data
                final_score = round((active_trip_data["medium_safety_score"] + active_trip_data["medium_eco_score"]) / 2)
                print(active_trip_data["events"])
                summary = str(active_trip_data["events"])  # Convert dict to string for summary
                
                # Generate LLM advice immediately
                trip_summary_for_llm = {
                    "eco_score": active_trip_data["medium_eco_score"],
                    "safety_score": active_trip_data["medium_safety_score"], 
                    "total_score": final_score,
                    "duration_min": round((asyncio.get_event_loop().time() - active_trip_data["start_time"]) / 60) if active_trip_data["start_time"] else 0,
                    "violations": [{"type": event, "count": count, "severity": "medium"} for event, count in active_trip_data["events"].items()]
                }
                
                try:
                    llm_advice = get_trip_advice(trip_summary_for_llm)
                except Exception as e:
                    print(f"Error generating LLM advice: {e}")
                    llm_advice = "Unable to generate driving advice at this time."
                
                # Save to Firebase
                trip_id = firebase.save_trip(final_score, summary)

                if trip_id:
                    trip_data_for_modal = {
                        "tripId": trip_id,
                        "score": final_score,
                        "summary": summary,
                        "llmAdvice": llm_advice
                    }
                    await websocket.send_json({"action": "trip_ended", "tripData": trip_data_for_modal})
                else:
                    # Handle offline case
                    trip_data_for_modal = {
                        "tripId": None,
                        "score": final_score,
                        "summary": summary + " (Trip not saved to cloud - offline)",
                        "llmAdvice": llm_advice
                    }
                    await websocket.send_json({"action": "trip_ended", "tripData": trip_data_for_modal})

    except Exception as e:
        print(f"WebSocket Error: {e}")
    finally:
        manager.disconnect(websocket)


# --- API Endpoints for Detector & Frontend ---

@app.post("/event")
async def receive_event_from_detector(request: Request):
    """Receives real-time data from the detector.py script."""
    if not active_trip_data["is_active"]:
        return {"status": "trip_inactive"}

    data = await request.json()
    
    # Store scores and reminder for final summary
    active_trip_data["last_safety_score"] = data.get("safety_score", 100)
    active_trip_data["last_eco_score"] = data.get("eco_score", 100)
    active_trip_data["medium_safety_score"] = (active_trip_data["medium_safety_score"] * active_trip_data["number_of_events"] + active_trip_data["last_safety_score"]) / (active_trip_data["number_of_events"] + 1)
    active_trip_data["medium_eco_score"] = (active_trip_data["medium_eco_score"] * active_trip_data["number_of_events"] + active_trip_data["last_eco_score"]) / (active_trip_data["number_of_events"] + 1)
    active_trip_data["number_of_events"] += 1
    # print(active_trip_data["medium_safety_score"], active_trip_data["medium_eco_score"], active_trip_data["number_of_events"])
    reminder = data.get("reminder")
    if reminder and reminder != "": # Only log actual events
        if reminder not in active_trip_data["events"]:
            active_trip_data["events"][reminder] = 1
        else:
            active_trip_data["events"][reminder] += 1

    # Broadcast to UI
    await manager.broadcast(json.dumps({"action": "trip_update", **data}))
    return {"status": "success"}

@app.get("/leaderboard")
async def get_leaderboard():
    """Provides leaderboard data to the frontend."""
    leaderboard_data = firebase.get_leaderboard()
    return JSONResponse(content=leaderboard_data)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
