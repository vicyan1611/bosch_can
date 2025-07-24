import can
import cantools
import os
import json
from cantools.database.can.signal import NamedSignalValue

# Paths
db_path = os.path.join('data', 'BOSCH_CAN.dbc')
log_path = os.path.join('data', 'CANWIN.asc')
output_json_path = os.path.join('data', 'decoded_can.json')

# Load DBC and ASC log
db = cantools.database.load_file(db_path)
log = can.ASCReader(log_path)

# signal = db.get_message_by_frame_id(0x91).get_signal_by_name('VSA_YAW_1')
# print(signal.scale, signal.offset)

# Ensure output folder exists
os.makedirs('output', exist_ok=True)

# Get set of known message IDs from DBC
known_ids = set(message.frame_id for message in db.messages)

# Helper: convert NamedSignalValue to str or int
def sanitize(decoded_dict):
    sanitized = {}
    for key, value in decoded_dict.items():
        if isinstance(value, NamedSignalValue):
            sanitized[key] = value.value # or: `value.value` to get the integer value
        else:
            sanitized[key] = value
    return sanitized

# Store decoded results
decoded_output = []

# Decode only known messages
for msg in log:
    if msg.arbitration_id in known_ids:
        try:
            decoded = db.decode_message(msg.arbitration_id, msg.data, decode_choices=False)
            # if 'VSA_YAW_1' in decoded:
            #     print(f"ID: {hex(msg.arbitration_id)}, VSA_YAW_1: {decoded['VSA_YAW_1']}")
            decoded_output.append({
                'Timestamp': msg.timestamp,
                'CAN_ID': hex(msg.arbitration_id),
                'Decoded': sanitize(decoded)
            })
        except Exception as e:
            print(f"Failed to decode {hex(msg.arbitration_id)}: {e}")
    else:
        # Skip unknown messages
        continue

# Write to JSON file
with open(output_json_path, 'w') as f:
    json.dump(decoded_output, f, indent=2)

print(f"✅ Decoding complete. Output saved to: {output_json_path}")
