# main.py

import math
import time
import csv
import matplotlib.pyplot as plt
from CANDataPackage import CANDataPackage
from DrivingScoreEvaluator import DrivingScoreEvaluator

def clamp(value, min_value, max_value):
    """Hàm phụ trợ để giới hạn một giá trị trong một khoảng nhất định."""
    return max(min_value, min(value, max_value))

# --- Example Usage ---
if __name__ == "__main__":
    # 1. Initialize the Evaluator
    evaluator = DrivingScoreEvaluator()

    print("Starting driving simulation...")
    current_sim_time = 0.0
    time_step = 0.1 # Simulate CAN data every 0.1 seconds
    TOTAL_SIM_DURATION = 300 # 5 minutes

    eco_scores_log = []
    eco_timestamps_log = []
    
    safety_scores_log = []
    safety_timestamps_log = []

    # Log all raw data + scores for CSV export
    full_data_log = []
    
    # Define headers for CSV
    csv_headers = [
        'timestamp',
        'ENG_DRIVER_REQ_TRQ_13C', 'ENG_SMART_ACCELE_PEDAL_POS_13C',
        'VSA_ABS_FL_WHEEL_SPEED', 'ENG_ENG_SPEED', 'CVT_GEAR_POSITION_IND_CVT',
        'ENG_IS_PROGRESS', 'VSA_LON_G', 'VSA_LAT_G', 'VSA_YAW_1', 'STR_ANGLE',
        'VSA_VSA_TCS_ACT', 'VSA_ABS_EBD_ACT',
        'eco_score', 'safety_score'
    ]

    # Simulate 5 minutes of driving data
    while current_sim_time <= TOTAL_SIM_DURATION:
        # Initialize base values for the current timestep
        sim_speed = 60 + math.sin(current_sim_time / 10) * 20
        sim_rpm = 1800 + math.sin(current_sim_time / 8) * 300
        sim_driver_req_trq = 50 + math.sin(current_sim_time / 7) * 30
        sim_pedal_pos = 30 + math.sin(current_sim_time / 6) * 15
        sim_gear = 'D'
        sim_is_progress = False
        sim_lon_g = 0.0
        sim_lat_g = 0.0
        sim_yaw = 0.0
        sim_str_angle = math.sin(current_sim_time / 5) * 20
        sim_vsa_tcs_act = False
        sim_abs_ebd_act = False

        # --- Scenario-based Event Injection ---

        # Smooth Driving
        if 0 <= current_sim_time < 30 or 150 <= current_sim_time < 180:
            sim_driver_req_trq = 40 + math.sin(current_sim_time / 15) * 20
            sim_pedal_pos = 25 + math.sin(current_sim_time / 12) * 10
            sim_speed = 40 + math.sin(current_sim_time / 10) * 10
            sim_rpm = 1500 + math.sin(current_sim_time / 10) * 200

        # Efficient RPM
        if 30 <= current_sim_time < 60 or 180 <= current_sim_time < 210:
            sim_rpm = 1500; sim_speed = 50; sim_driver_req_trq = 30; sim_pedal_pos = 20
        
        # Inefficient High RPM
        if 60 <= current_sim_time < 90 or 210 <= current_sim_time < 240:
            sim_rpm = 4000; sim_speed = 45; sim_driver_req_trq = 70; sim_pedal_pos = 60

        # Idling Management
        if 90 <= current_sim_time < 95 or 240 <= current_sim_time < 245:
            sim_speed = 0.05; sim_rpm = 800; sim_is_progress = False; sim_driver_req_trq = 0; sim_pedal_pos = 0
        if 95 <= current_sim_time < 100 or 245 <= current_sim_time < 250:
            sim_speed = 0.05; sim_rpm = 0; sim_is_progress = True; sim_driver_req_trq = 0; sim_pedal_pos = 0

        # Longitudinal G-Force Events
        if 10 < current_sim_time < 10.2: sim_lon_g = -8.0
        if 11 < current_sim_time < 11.2: sim_lon_g = 6.0
        if 160 < current_sim_time < 160.2: sim_lon_g = -7.5
        if 161 < current_sim_time < 161.2: sim_lon_g = 6.8

        # Lateral Maneuver Events
        if 40 < current_sim_time < 40.3:
            sim_lat_g = 4.5; sim_yaw = 35; sim_speed = 70
        if 190 < current_sim_time < 190.3:
            sim_lat_g = 4.0; sim_yaw = 30; sim_speed = 60
        
        # Jerky Steering Events 
        if 45 < current_sim_time < 45.1:
            generated_angle = 200 * math.sin(current_sim_time * 15)
            sim_str_angle = clamp(generated_angle, -103.0, 103.0)
        if 195 < current_sim_time < 195.1:
            generated_angle = 250 * math.sin(current_sim_time * 12)
            sim_str_angle = clamp(generated_angle, -103.0, 103.0)

        # Safety System Activation Events
        if 70 < current_sim_time < 70.3: sim_abs_ebd_act = True
        if 220 < current_sim_time < 220.3: sim_vsa_tcs_act = True

        # Cumulative Unsafe Driving Events
        if 120 < current_sim_time < 120.2: sim_lon_g = 4.0
        if 120.5 < current_sim_time < 120.7:
            generated_angle = 150 * math.sin(current_sim_time * 10)
            sim_str_angle = clamp(generated_angle, -103.0, 103.0)
        if 121.0 < current_sim_time < 121.3: sim_lon_g = -6.0
        if 121.5 < current_sim_time < 121.8:
            sim_lat_g = 3.5; sim_yaw = 25; sim_speed = 40
        if 122.0 < current_sim_time < 122.2: sim_abs_ebd_act = True

        # Create the CAN data packet
        can_packet = CANDataPackage(
            timestamp=current_sim_time,
            ENG_DRIVER_REQ_TRQ_13C=sim_driver_req_trq,
            ENG_SMART_ACCELE_PEDAL_POS_13C=sim_pedal_pos,
            VSA_ABS_FL_WHEEL_SPEED=sim_speed,
            ENG_ENG_SPEED=sim_rpm,
            CVT_GEAR_POSITION_IND_CVT=sim_gear,
            ENG_IS_PROGRESS=sim_is_progress,
            VSA_LON_G=sim_lon_g,
            VSA_LAT_G=sim_lat_g,
            VSA_YAW_1=sim_yaw,
            STR_ANGLE=sim_str_angle,
            VSA_VSA_TCS_ACT=sim_vsa_tcs_act,
            VSA_ABS_EBD_ACT=sim_abs_ebd_act
        )
    
        eco_score, safety_score = evaluator.process_can_data(can_packet)
    
        current_data_row = can_packet.to_dict()
        current_data_row['eco_score'] = eco_score
        current_data_row['safety_score'] = safety_score
        full_data_log.append(current_data_row)
    
        if eco_score is not None:
            eco_scores_log.append(eco_score)
            eco_timestamps_log.append(current_sim_time)
            
        if safety_score is not None:
            safety_scores_log.append(safety_score)
            safety_timestamps_log.append(current_sim_time)

        current_sim_time += time_step

    print("\nSimulation complete.")
    
    # --- Export Data to CSV ---
    csv_file_path = 'driving_simulation_data.csv'
    with open(csv_file_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=csv_headers)
        writer.writeheader()
        writer.writerows(full_data_log)
    print(f"Simulation data exported to {csv_file_path}")

    # --- Plotting ---
    plt.figure(figsize=(14, 7))

    plt.subplot(2, 1, 1)
    plt.plot(eco_timestamps_log, eco_scores_log, label='Eco Score', color='green')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Eco Score (0-100)')
    plt.title('Eco-Friendly Driving Score Evolution')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(safety_timestamps_log, safety_scores_log, label='Safety Score', color='red')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Safety Score (0-100)')
    plt.title('Safety Driving Score Evolution')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()