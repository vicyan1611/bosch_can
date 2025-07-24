import json
import os

log_json_path = "data/decoded_can.json"
target_can_id = "0x156".lower()

with open(log_json_path, "r", encoding="utf-8") as f:
    data = json.load(f)


filtered_messages = [msg for msg in data if msg.get("CAN_ID", "").lower() == target_can_id]

os.makedirs("messages", exist_ok=True)

with open(f"messages/{target_can_id}.json", "w", encoding="utf-8") as out_file:
    json.dump(filtered_messages, out_file, indent=4)
