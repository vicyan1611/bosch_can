import can
import cantools
import os
import json
import csv
from cantools.database.can.signal import NamedSignalValue

# Paths
db_path = os.path.join('data', 'BOSCH_CAN.dbc')
log_path = os.path.join('data', 'CANWIN.asc')
output_csv_path = os.path.join('data', 'decoded_can.csv')

# Load DBC and ASC log
db = cantools.database.load_file(db_path)
log = can.ASCReader(log_path)

# Ensure output folder exists
os.makedirs('output', exist_ok=True)

# Set of known CAN IDs
known_ids = set(message.frame_id for message in db.messages)

# Target signals and initialize last known values
target_signals = [
    'ENG_DRIVER_REQ_TRQ_13C',
    'ENG_SMART_ACCELE_PEDAL_POS_13C',
    'VSA_ABS_FL_WHEEL_SPEED',
    'ENG_ENG_SPEED',
    'CVT_GEAR_POSITION_IND_CVT',
    'ENG_IS_PROGRESS',
    'VSA_LON_G',
    'VSA_LAT_G',
    'VSA_YAW_1',
    'STR_ANGLE',
    'VSA_VSA_TCS_ACT',
    'VSA_ABS_EBD_ACT'
]
last_known = {sig: '' for sig in target_signals}  # You can use `None` instead of '' if preferred

# Helper: convert NamedSignalValue to int
def sanitize(decoded_dict):
    sanitized = {}
    for key, value in decoded_dict.items():
        if isinstance(value, NamedSignalValue):
            sanitized[key] = value.value
        else:
            sanitized[key] = value
    return sanitized

# Open CSV writer
with open(output_csv_path, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=['timestamp'] + target_signals)
    writer.writeheader()

    # Decode and write to CSV
    for msg in log:
        if msg.arbitration_id in known_ids:
            try:
                decoded = db.decode_message(msg.arbitration_id, msg.data, decode_choices=False)
                decoded = sanitize(decoded)

                # Update last known values
                for sig in target_signals:
                    if sig in decoded:
                        last_known[sig] = decoded[sig]

                # Write row using current timestamp and last known values
                row = {'timestamp': msg.timestamp}
                row.update(last_known)
                writer.writerow(row)

            except Exception as e:
                print(f"⚠️ Failed to decode {hex(msg.arbitration_id)}: {e}")
                continue

print(f"✅ CSV decoding complete. Output saved to: {output_csv_path}")
