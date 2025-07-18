# simulate_can_messages.py

import pandas as pd
import can
import cantools
import time
from typing import Literal
from ecu_simulator import *

# --- Configuration ---
CSV_FILE_PATH = 'scoring/driving_simulation_data.csv'  # Make sure this matches your CSV file name
DBC_FILE_PATH = 'BOSCH_CAN.dbc'     # Replace with the actual path to your DBC file
BUS_TYPE = 'socketcan'                  # 'virtual', 'socketcan', 'pcan', etc.
CHANNEL = 'vcan0'                     # 'vcan0' for virtual, or specify your CAN interface


# Function to map gear indicator to literal for create_cvt_191_message
def map_gear_indicator_to_literal(gear_indicator: str) -> GearSelection:
    if gear_indicator == 'P': return 'P'
    elif gear_indicator == 'R': return 'R'
    elif gear_indicator == 'N': return 'N'
    elif gear_indicator == 'D': return 'D'
    elif gear_indicator == 'S': return 'S' # Assuming 'S' might be an output for sport mode
    elif gear_indicator == 'L': return 'L' # Assuming 'L' might be an output for low gear
    else: return 'D' # Default to Drive if unknown

def simulate_can_traffic(csv_file: str, dbc_file: str, bus_type: str, channel: str):
    print(f"Loading DBC file from: {dbc_file}")
    db = cantools.database.load_file(dbc_file)
    print("DBC file loaded successfully.")

    print(f"Loading data from CSV file: {csv_file}")
    try:
        df = pd.read_csv(csv_file)
        print("CSV file loaded successfully.")
        print(f"Number of data points: {len(df)}")
    except FileNotFoundError:
        print(f"Error: CSV file not found at {csv_file}")
        return
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        return

    print(f"Initializing CAN bus: Type={bus_type}, Channel={channel}")
    try:
        bus = can.interface.Bus(bustype=bus_type, channel=channel, bitrate=500000)
        print("CAN bus initialized.")
    except Exception as e:
        print(f"Error initializing CAN bus: {e}")
        print("Please ensure your CAN interface is properly configured (e.g., 'sudo modprobe vcan; sudo ip link add dev vcan0 type vcan; sudo ip link set up vcan0' for virtual CAN on Linux)")
        return

    alive_counter = 0
    previous_timestamp = 0.0

    print("\nStarting CAN message simulation...")
    for index, row in df.iterrows():
        current_time = row['timestamp']

        # Calculate time step for realistic delay
        if index > 0:
            time_difference = current_time - previous_timestamp
            if time_difference > 0:
                time.sleep(time_difference)
        previous_timestamp = current_time

        alive_counter = (alive_counter + 1) % 16 # Typically 4-bit counter (0-15)

        print(f"--- Time: {current_time:.2f}s ---")

        # Create and send ENG_13C message
        eng_13c_msg = create_eng_13c_message(
            db=db,
            alive_counter=alive_counter,
            pedal_pos_percent=row['ENG_SMART_ACCELE_PEDAL_POS_13C'],
            driver_torque_nm=row['ENG_DRIVER_REQ_TRQ_13C'],
            # engine_torque_nm and in_reverse are not in CSV, using defaults
        )
        try:
            bus.send(eng_13c_msg)
            # print(f"Sent ENG_13C: {eng_13c_msg}")
        except Exception as e:
            print(f"Failed to send ENG_13C: {e}")

        # Create and send VSA_1D0 message
        vsa_1d0_msg = create_vsa_1d0_message(
            db=db,
            fl_speed_kph=row['VSA_ABS_FL_WHEEL_SPEED']
            # Other wheel speeds default to FL_WHEEL_SPEED if not explicitly provided
        )
        try:
            bus.send(vsa_1d0_msg)
            # print(f"Sent VSA_1D0: {vsa_1d0_msg}")
        except Exception as e:
            print(f"Failed to send VSA_1D0: {e}")

        # Create and send ENG_17C message
        eng_17c_msg = create_eng_17c_message(
            db=db,
            alive_counter=alive_counter,
            engine_speed_rpm=row['ENG_ENG_SPEED'],
            is_progress=bool(row['ENG_IS_PROGRESS']) # Convert 0/1 to boolean
        )
        try:
            bus.send(eng_17c_msg)
            # print(f"Sent ENG_17C: {eng_17c_msg}")
        except Exception as e:
            print(f"Failed to send ENG_17C: {e}")

        # Create and send CVT_191 message
        # Convert the string gear to the literal expected by the function
        gear_literal = map_gear_indicator_to_literal(row['CVT_GEAR_POSITION_IND_CVT'])
        cvt_191_msg = create_cvt_191_message(
            db=db,
            alive_counter=alive_counter,
            cvt_gear_position_ind_cvt=1 if gear_literal == 'D' else 0, # Assuming 'D' is 1, otherwise 0 based on sim logic
            gear=gear_literal
            # current_ratio and target_ratio use defaults as they are not in CSV
        )
        try:
            bus.send(cvt_191_msg)
            # print(f"Sent CVT_191: {cvt_191_msg}")
        except Exception as e:
            print(f"Failed to send CVT_191: {e}")


        # Create and send VSA_091 message
        vsa_091_msg = create_vsa_091_message(
            db=db,
            alive_counter=alive_counter,
            yaw_rate=row['VSA_YAW_1'],
            steering_angle=row['STR_ANGLE'],
            lateral_g=row['VSA_LAT_G'],
            longitudinal_g=row['VSA_LON_G']
        )
        try:
            bus.send(vsa_091_msg)
            # print(f"Sent VSA_091: {vsa_091_msg}")
        except Exception as e:
            print(f"Failed to send VSA_091: {e}")

        # # Create and send VSA_255 message
        # vsa_255_msg = create_vsa_255_message(
        #     db=db,
        #     alive_counter=alive_counter,
        #     vsa_tcs_act=bool(row['VSA_VSA_TCS_ACT']), # Convert 0/1 to boolean
        #     abs_ebd_act=bool(row['VSA_ABS_EBD_ACT']) # Convert 0/1 to boolean
        # )
        # try:
        #     bus.send(vsa_255_msg)
        #     # print(f"Sent VSA_255: {vsa_255_msg}")
        # except Exception as e:
        #     print(f"Failed to send VSA_255: {e}")

        # Create and send VSA_1A4 message
        vsa_1a4_msg = create_vsa_1a4_message(
            db=db,
            alive_counter=alive_counter,
            vsa_tcs_act=bool(row['VSA_VSA_TCS_ACT']),
            abs_ebd_act=bool(row['VSA_ABS_EBD_ACT'])
        )
        try:
            bus.send(vsa_1a4_msg)
            # print(f"Sent VSA_1A4: {vsa_1a4_msg}")
        except Exception as e:
            print(f"Failed to send VSA_1A4: {e}")

        print(f"Messages for time {current_time:.2f}s sent.")

    bus.shutdown()
    print("\nCAN message simulation complete.")

if __name__ == "__main__":
    # Ensure you have your 'honda_civic.dbc' file in the same directory
    # or provide the full path to it.
    # Replace 'simulated_data.csv' with the actual name of your exported CSV file.
    simulate_can_traffic(CSV_FILE_PATH, DBC_FILE_PATH, BUS_TYPE, CHANNEL)