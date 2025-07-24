import cantools
import json
import os

# Load the DBC file
dbc_path = 'data/BOSCH_CAN.dbc'
db = cantools.database.load_file(dbc_path)

# Create output folder if it doesn't exist
output_folder = 'data'
os.makedirs(output_folder, exist_ok=True)
output_file = os.path.join(output_folder, 'signals_info.json')

# Your target signal names
signal_names = [
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

# Extract signal information
signal_info = {}

for message in db.messages:
    for signal in message.signals:
        if signal.name in signal_names:
            signal_info[signal.name] = {
                "message_name": message.name,
                "message_id": hex(message.frame_id),
                "start_bit": signal.start,
                "length": signal.length,
                "factor": signal.scale,
                "offset": signal.offset,
                "min": signal.minimum,
                "max": signal.maximum,
                "unit": signal.unit,
                "receivers": signal.receivers
            }

# Save to JSON
with open(output_file, 'w') as f:
    json.dump(signal_info, f, indent=2)

print(f"Signal info saved to {output_file}")
