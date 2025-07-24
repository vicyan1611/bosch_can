import os
import google.generativeai as genai
from dotenv import load_dotenv
from typing import Dict, List

# Load environment variables from .env
load_dotenv()

# Configure Gemini API
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise ValueError("Missing GEMINI_API_KEY in .env file")

genai.configure(api_key=api_key)

# Sample trip summary (replace this with real-time data in production)
trip_summary: Dict = {
    "eco_score": 64,
    "safety_score": 80,
    "total_score": 72,
    "duration_min": 25,
    "violations": [
        {"type": "Revving engine at high RPM", "count": 4, "severity": "medium"},
        {"type": "Shifting gears repeatedly", "count": 3, "severity": "low"},
        {"type": "Excessive idling", "count": 5, "severity": "low"},
        {"type": "Engine temperature exceeded limit", "count": 2, "severity": "high"}
    ]
}

def generate_driver_prompt(summary: Dict) -> str:
    eco_score = summary.get("eco_score", "N/A")
    safety_score = summary.get("safety_score", "N/A")
    total_score = summary.get("total_score", "N/A")
    duration = summary.get("duration_min", "N/A")
    violations: List[Dict] = summary.get("violations", [])

    if violations:
        violation_text = "\n".join(
            f"- {v['type']} ({v['count']} times) – severity: {v['severity']}"
            for v in violations
        )
    else:
        violation_text = "No driving violations were detected."

    prompt = (
        f"You are an automotive driving assistant AI. A trip has just completed with the following data:\n"
        f"- Duration: {duration} minutes\n"
        f"- Eco Score: {eco_score}/100\n"
        f"- Safety Score: {safety_score}/100\n"
        f"- Total Score: {total_score}/100\n"
        f"- Driving violations detected:\n{violation_text}\n\n"
        f"Please generate a personalized advice summary for the driver to improve future driving behavior. Include:\n"
        f"All the output will be displayed on website so answer straight to work\n"
        f"Do not mention again the violations\n"
        f"Friendly overall comment\n"
        f"Key mistakes and their impact\n"
        f"Practical advice for ECO and SAFETY improvement\n"
        f"Tone: supportive, concise, but technically accurate."
    )
    return prompt


def get_trip_advice(summary: Dict) -> str:
    prompt = generate_driver_prompt(summary)

    try:
        model = genai.GenerativeModel("gemini-2.0-flash") 
        response = model.generate_content(prompt)
        return response.text.strip()
    except Exception as e:
        return f"[ERROR] Failed to generate advice: {e}"
