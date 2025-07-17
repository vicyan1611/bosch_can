import math
import time
import csv
import matplotlib.pyplot as plt # Make sure this import is present
from CANDataPackage import CANDataPackage
from DrivingScoreEvaluator import DrivingScoreEvaluator

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
    
    # Define headers for CSV (all CAN signals + calculated scores)
    # This needs to be comprehensive based on CANSIGNALS attributes
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
        sim_speed = 60 + math.sin(current_sim_time / 10) * 20 # Fluctuating speed for normal driving
        sim_rpm = 1800 + math.sin(current_sim_time / 8) * 300 # Fluctuating RPM for normal driving
        sim_driver_req_trq = 50 + math.sin(current_sim_time / 7) * 30
        sim_pedal_pos = 30 + math.sin(current_sim_time / 6) * 15
        sim_gear = 'D'
        sim_is_progress = False
        sim_lon_g = 0.0
        sim_lat_g = 0.0
        sim_yaw = 0.0
        sim_str_angle = math.sin(current_sim_time / 5) * 20 # Gentle steering
        sim_vsa_tcs_act = False
        sim_abs_ebd_act = False

        # --- Scenario-based Event Injection ---

        # Scenario 1: Smooth Acceleration and Deceleration (default sinusoidal motion is generally smooth)
        # To emphasize smoothness, ensure no sharp changes
        if 0 <= current_sim_time < 30 or 150 <= current_sim_time < 180:
            # Very gentle changes
            sim_driver_req_trq = 40 + math.sin(current_sim_time / 15) * 20
            sim_pedal_pos = 25 + math.sin(current_sim_time / 12) * 10
            sim_speed = 40 + math.sin(current_sim_time / 10) * 10
            sim_rpm = 1500 + math.sin(current_sim_time / 10) * 200

        # Scenario 2: Optimized Engine RPM Management (periods of efficient RPM)
        # Default sim_rpm is already somewhat optimized. To show bad RPM, we'll introduce it.
        # Good RPM period: 30-60s, 180-210s
        if 30 <= current_sim_time < 60 or 180 <= current_sim_time < 210:
            sim_rpm = 1500 # Keep RPM low for given speed
            sim_speed = 50 # Consistent speed
            sim_driver_req_trq = 30
            sim_pedal_pos = 20
        
        # Scenario 2 (Bad): High RPM for given speed
        if 60 <= current_sim_time < 90 or 210 <= current_sim_time < 240:
            sim_rpm = 4000 # High RPM
            sim_speed = 45 # Moderate speed, indicating inefficient gear/driving
            sim_driver_req_trq = 70
            sim_pedal_pos = 60


        # Scenario 3: Efficient Idling Management
        # Idling without idle-stop (inefficient)
        if 90 <= current_sim_time < 95 or 240 <= current_sim_time < 245: # 5 seconds of idling
            sim_speed = 0.05 # Below MIN_DRIVING_SPEED_FOR_IDLE
            sim_rpm = 800 # Above MIN_IDLE_RPM_THRESHOLD
            sim_is_progress = False
            sim_driver_req_trq = 0
            sim_pedal_pos = 0

        # Idling with idle-stop (efficient)
        if 95 <= current_sim_time < 100 or 245 <= current_sim_time < 250: # 5 seconds of idle-stop
            sim_speed = 0.05
            sim_rpm = 0 # Engine off
            sim_is_progress = True # Idle-stop active
            sim_driver_req_trq = 0
            sim_pedal_pos = 0


        # Scenario 1 (Safety): Longitudinal G-Force Control (Hard Accel/Brake)
        if 10 < current_sim_time < 10.2: # Hard brake
            sim_lon_g = -8.0
        if 11 < current_sim_time < 11.2: # Hard accel (after brake)
            sim_lon_g = 6.0
        if 160 < current_sim_time < 160.2: # Another hard brake
            sim_lon_g = -7.5
        if 161 < current_sim_time < 161.2: # Another hard accel
            sim_lon_g = 6.8


        # Scenario 2 (Safety): Lateral Maneuver and Steering Control (Aggressive Cornering/Jerky Steering)
        if 40 < current_sim_time < 40.3: # Aggressive cornering
            sim_lat_g = 4.5
            sim_yaw = 35
            sim_speed = 70 # Simulate at higher speed
        if 45 < current_sim_time < 45.1: # Jerky steering
            sim_str_angle = 200 * math.sin(current_sim_time * 15) # Very rapid changes
        if 190 < current_sim_time < 190.3: # Another aggressive cornering
            sim_lat_g = 4.0
            sim_yaw = 30
            sim_speed = 60
        if 195 < current_sim_time < 195.1: # Another jerky steering
            sim_str_angle = 250 * math.sin(current_sim_time * 12)


        # Scenario 3 (Safety): Safety System Activation Prevention
        if 70 < current_sim_time < 70.3: # ABS/VSA activation
            sim_abs_ebd_act = True
        if 220 < current_sim_time < 220.3: # Another ABS/VSA activation
            sim_vsa_tcs_act = True


        # Scenario 4 (Safety): Cumulative Unsafe Driving Events (multiple events close together)
        if 120 < current_sim_time < 120.2: # Hard accel
            sim_lon_g = 4.0
        if 120.5 < current_sim_time < 120.7: # Jerky steering
            sim_str_angle = 150 * math.sin(current_sim_time * 10)
        if 121.0 < current_sim_time < 121.3: # Hard brake
            sim_lon_g = -6.0
        if 121.5 < current_sim_time < 121.8: # Aggressive cornering
            sim_lat_g = 3.5
            sim_yaw = 25
            sim_speed = 40
        if 122.0 < current_sim_time < 122.2: # ABS activation
            sim_abs_ebd_act = True


        # Create the CAN data packet for the current timestep
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

        # 3. Process the CAN data and get scores
        eco_score, safety_score = evaluator.process_can_data(can_packet)

        # Prepare row for CSV log
        current_data_row = can_packet.to_dict()
        current_data_row['eco_score'] = eco_score
        current_data_row['safety_score'] = safety_score
        full_data_log.append(current_data_row)

        # Log for console and plotting
        if eco_score is not None:
            eco_scores_log.append(eco_score)
            eco_timestamps_log.append(current_sim_time)
            print(f"Time: {current_sim_time:.1f}s | Eco Score: {eco_score:.2f}")

        if safety_score is not None:
            safety_scores_log.append(safety_score)
            safety_timestamps_log.append(current_sim_time)
            print(f"Time: {current_sim_time:.1f}s | Safety Score: {safety_score:.2f}")

        current_sim_time += time_step
        # Optional: time.sleep(time_step) # Uncomment to simulate real-time delay

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

    plt.subplot(2, 1, 1) # 2 rows, 1 column, first plot
    plt.plot(eco_timestamps_log, eco_scores_log, label='Eco Score', color='green')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Eco Score (0-100)')
    plt.title('Eco-Friendly Driving Score Evolution')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2) # 2 rows, 1 column, second plot
    plt.plot(safety_timestamps_log, safety_scores_log, label='Safety Score', color='red')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Safety Score (0-100)')
    plt.title('Safety Driving Score Evolution')
    plt.legend()
    plt.grid(True)

    plt.tight_layout() # Adjust layout to prevent overlapping
    plt.show()
