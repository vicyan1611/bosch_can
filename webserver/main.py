# main.py
from fastapi import FastAPI, WebSocket, Request
from fastapi.responses import HTMLResponse
import uvicorn
import asyncio

app = FastAPI()

# A list to hold active WebSocket connections
connections: list[WebSocket] = []

html = ""
with open("index.html", "r") as f:
    html = f.read()

# Endpoint to serve the HTML page
@app.get("/")
async def get():
    return HTMLResponse(html)

# WebSocket endpoint that the frontend will connect to
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connections.append(websocket)
    try:
        while True:
            # Keep the connection alive
            await websocket.receive_text()
    except Exception:
        connections.remove(websocket)

# A new endpoint for detector.py to post data to
@app.post("/event")
async def send_event(request: Request):
    data = await request.json() # e.g., {"score": 85, "reminder": "Sudden brake detected"}
    # Broadcast the data to all connected web clients
    for connection in connections:
        await connection.send_json(data)
    return {"status": "success", "data_sent": data}

if __name__ == "__main__":
    # Runs the server on Pi's IP address, accessible from network
    uvicorn.run(app, host="0.0.0.0", port=8000)