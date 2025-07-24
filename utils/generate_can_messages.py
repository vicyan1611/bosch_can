import pandas as pd
import cantools
import can
import time
import json

from ecu_simulator import (
    create_eng_13c_message,
    create_vsa_1d0_message,
    create_cvt_191_message,
    GearSelection,
    create_eng_17c_message,
    create_vsa_091_message
)

# Load the DBC file
db = cantools.database.load_file('BOSCH_CAN.dbc')

def generate_and_log_can_messages(csv_file_path: str, output_log_path: str):
    """
    Reads a CSV file, generates CAN messages from its data, and logs
    the arbitration ID and data of each message to an output file.
    """
    df = pd.read_csv(csv_file_path)

    # Prepare the output log file
    with open(output_log_path, 'w') as log_file:
        log_file.write("# CAN Message Log generated from CSV data\n")
        log_file.write("# Format: <timestamp_sec> <arbitration_id_hex> <data_hex_string>\n")
        
        # Keep track of alive counters
        alive_counter_13c = 0
        alive_counter_191 = 0
        alive_counter_17c = 0
        alive_counter_091 = 0

        # Iterate through each row of the DataFrame
        for index, row in df.iterrows():
            current_timestamp = row['timestamp']

            # --- Create ENG_13C message ---
            # Signals: ENG_DRIVER_REQ_TRQ_13C, ENG_SMART_ACCELE_PEDAL_POS_13C
            eng_13c_msg = create_eng_13c_message(
                db=db,
                alive_counter=alive_counter_13c,
                pedal_pos_percent=row['pedal_pos'],
                driver_torque_nm=row['trq_req'],
                engine_torque_nm=row['trq_req'] # Assuming actual engine torque matches requested for simplicity
            )
            log_file.write(f"{current_timestamp:.6f} {eng_13c_msg.arbitration_id:03X} {eng_13c_msg.data.hex()}\n")
            alive_counter_13c = (alive_counter_13c + 1) % 4 # Increment and wrap around 0-3

            # --- Create VSA_1D0 message ---
            # Signal: VSA_ABS_FL_WHEEL_SPEED (using this for overall speed)
            vsa_1d0_msg = create_vsa_1d0_message(
                db=db,
                fl_speed_kph=row['speed'],
                fr_speed_kph=row['speed'],
                rl_speed_kph=row['speed'],
                rr_speed_kph=row['speed']
            )
            log_file.write(f"{current_timestamp:.6f} {vsa_1d0_msg.arbitration_id:03X} {vsa_1d0_msg.data.hex()}\n")


            # --- Create CVT_191 message ---
            # Signal: CVT_GEAR_POSITION_IND_CVT (use a simple mapping or default)
            # You might need to map your numerical 'gear' from CSV to GearSelection Literal
            # For now, let's assume 'D' for driving, 'N' for idle, 'R' for reverse if speed is negative
            current_speed = row['speed']
            if current_speed > 0.1:
                gear_selection: GearSelection = 'D'
            elif current_speed < -0.1: # Assuming negative speed implies reverse
                gear_selection = 'R'
            else:
                gear_selection = 'N' # Or 'P' if explicitly parked/stopped for long

            cvt_191_msg = create_cvt_191_message(
                db=db,
                gear=gear_selection,
                alive_counter=alive_counter_191,
                current_ratio=2.5, # Placeholder: provide realistic ratio if available
                target_ratio=2.5   # Placeholder
            )
            # Manually set CVT_GEAR_POSITION_IND_CVT based on your logic if needed
            # For simplicity, I'll encode it as 0 here, or you could read from CSV
            decoded_signals = db.decode(cvt_191_msg.arbitration_id, cvt_191_msg.data)
            decoded_signals['CVT_GEAR_POSITION_IND_CVT'] = row['gear'] # Use actual gear from CSV
            cvt_191_msg.data = db.get_message_by_name('CVT_191').encode(decoded_signals)

            log_file.write(f"{current_timestamp:.6f} {cvt_191_msg.arbitration_id:03X} {cvt_191_msg.data.hex()}\n")
            alive_counter_191 = (alive_counter_191 + 1) % 4

            # --- Create ENG_17C message ---
            # Signals: ENG_ENG_SPEED, ENG_IS_PROGRESS
            # For ENG_IS_PROGRESS, map 'True' from CSV to 1, 'False' to 0
            eng_17c_msg = create_eng_17c_message(
                db=db,
                engine_speed_rpm=row['rpm'],
                alive_counter=alive_counter_17c,
                is_brake_pedal_pressed=(row['lon_g'] < -1.0) # Simple heuristic: if strong negative G, assume brake
            )
            # Manually set ENG_IS_PROGRESS based on your CSV value
            decoded_signals_17c = db.decode(eng_17c_msg.arbitration_id, eng_17c_msg.data)
            decoded_signals_17c['ENG_IS_PROGRESS'] = 1 if row['is_progress'] else 0
            eng_17c_msg.data = db.get_message_by_name('ENG_17C').encode(decoded_signals_17c)

            log_file.write(f"{current_timestamp:.6f} {eng_17c_msg.arbitration_id:03X} {eng_17c_msg.data.hex()}\n")
            alive_counter_17c = (alive_counter_17c + 1) % 4

            # --- Create VSA_091 message ---
            # Signals: VSA_LON_G, VSA_LAT_G, VSA_YAW_1, VSA_DEGC (steering_angle)
            vsa_091_msg = create_vsa_091_message(
                db=db,
                alive_counter=alive_counter_091,
                yaw_rate=row['yaw'],
                steering_angle=row['steering_angle'],
                lateral_g=row['lat_g'],
                longitudinal_g=row['lon_g']
            )
            log_file.write(f"{current_timestamp:.6f} {vsa_091_msg.arbitration_id:03X} {vsa_091_msg.data.hex()}\n")
            alive_counter_091 = (alive_counter_091 + 1) % 4

            print(f"Processed timestamp {current_timestamp:.2f}s and logged messages.")

    print(f"Successfully generated CAN message log to {output_log_path}")

if __name__ == '__main__':
    
    csv_input_file = 'scoring/driving_simulation_data.csv'
    can_log_output_file = 'simulated_can_bus_log.txt'

    generate_and_log_can_messages(csv_input_file, can_log_output_file)