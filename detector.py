import cantools
import can
import time

# --- Configuration ---
DBC_FILE = 'BOSCH_CAN.dbc'
CAN_INTERFACE = 'vcan0'
ECU_MESSAGE = 'METER_294'
SPEED_SIGNAL_NAME = 'METER_ODO_DATA'


try:
  db = cantools.db.load_file(DBC_FILE)
  bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
  print(f"Detector started. Listening on {CAN_INTERFACE}...")
except FileNotFoundError:
  print(f"Error: DBC file '{DBC_FILE}' not found.")
  exit()

for msg in bus:
  decoded_data = db.decode_message(msg.arbitration_id, msg.data)
  print(f"Decoded data: {decoded_data}")
