import math
import time # For simulation of timestamps
from CircularBuffer import CircularBuffer

class DrivingScoreEvaluator:
    def __init__(self, config=None, log_file_path='Simulating/logs/driving_events_log.txt'):
        # --- Configuration (Tunable Parameters) ---
        self.config = {
            # Window Durations (seconds)
            'ECO_WINDOW_DURATION_SEC': 60,
            'SAFETY_WINDOW_DURATION_SEC': 10,
            'EVENT_CHECK_HISTORY_SEC': 1, # For rate-of-change/duration checks

            # Calculation Intervals (seconds)
            'ECO_CALC_INTERVAL_SEC': 1.0,
            'SAFETY_CALC_INTERVAL_SEC': 0.2,

            # Eco-Friendly Thresholds & Weights
            'MIN_SPEED_FOR_RPM_RATIO_CALC': 5.0, # km/h
            'HIGH_RPM_THRESHOLD': 2500, # rpm
            'MIN_DRIVING_SPEED_FOR_IDLE': 0.1, # km/h
            'MIN_IDLE_RPM_THRESHOLD': 300, # rpm
            'AGGRESSIVE_ACCEL_DECEL_THRESHOLD_KMPHPS': 5.0, # km/h/s

            'k1_accel_jt': 0.01,  'k2_accel_jp': 0.05,  'k3_accel_vs': 0.001, 'k4_accel_eagg': 0.5,
            'w1_accel': 0.25, 'w2_accel': 0.25, 'w3_accel': 0.25, 'w4_accel': 0.25,
            
            'k5_rpm_ratio': 0.001, 'k6_rpm_high': 0.1,
            'w5_rpm': 0.7, 'w6_rpm': 0.3,

            'k7_idle_time': 0.1, 'k8_idle_is': 0.1,
            'w7_idle': 0.9, 'w8_idle': 0.1,

            'W_accel_overall': 0.4, 'W_rpm_overall': 0.3, 'W_idle_overall': 0.3, # Overall weights

            # Safety Thresholds & Weights
            'G_ACCEL_THRESHOLD_STATIC': 3.9, # m/s^2 (approx 0.4G)
            'G_BRAKE_THRESHOLD_STATIC': 5.8, # m/s^2 (approx 0.6G)
            'G_MAX_EXPECTED_ACCEL': 9.8, # m/s^2 (approx 1.0G, for normalization)
            'G_MAX_EXPECTED_BRAKE': 9.8, # m/s^2

            'LAT_G_THRESHOLD_DYNAMIC_LOW_SPEED': 2.9, # m/s^2 (e.g., at 0-20 km/h)
            'LAT_G_THRESHOLD_DYNAMIC_HIGH_SPEED': 4.9, # m/s^2 (e.g., at >80 km/h)
            'YAW_THRESHOLD_DYNAMIC_LOW_SPEED': 20, # deg/s
            'YAW_THRESHOLD_DYNAMIC_HIGH_SPEED': 40, # deg/s
            'MAX_LAT_G': 9.8, # m/s^2
            'MAX_YAW_RATE': 80, # deg/s

            'STEERING_RATE_THRESHOLD': 120, # deg/s
            'MAX_STEERING_RATE': 756, # deg/s (e.g., lock-to-lock in ~1 sec)

            'C_LonG': 25,
            'C_LatG_Yaw': 35,
            'C_Steering': 15,
            'C_Intervention': 70,

            'ALPHA_LON_G': 1.5,
            'BETA_LAT_G_YAW': 1.0,
            'GAMMA_STEERING': 1.0,
            
            'EVENT_COOLDOWN_SEC': 0.5,
        }
        if config:
            self.config.update(config)

        # --- Circular Buffers for each signal ---
        self.eco_buffers = {
            'trq_req': CircularBuffer(self.config['ECO_WINDOW_DURATION_SEC']),
            'pedal_pos': CircularBuffer(self.config['ECO_WINDOW_DURATION_SEC']),
            'speed': CircularBuffer(self.config['ECO_WINDOW_DURATION_SEC']),
            'rpm': CircularBuffer(self.config['ECO_WINDOW_DURATION_SEC']),
            'gear': CircularBuffer(self.config['ECO_WINDOW_DURATION_SEC']),
            'is_progress': CircularBuffer(self.config['ECO_WINDOW_DURATION_SEC']),
        }

        self.safety_buffers = {
            'lon_g': CircularBuffer(self.config['SAFETY_WINDOW_DURATION_SEC']),
            'lat_g': CircularBuffer(self.config['SAFETY_WINDOW_DURATION_SEC']),
            'yaw': CircularBuffer(self.config['SAFETY_WINDOW_DURATION_SEC']),
            'steering_angle': CircularBuffer(self.config['SAFETY_WINDOW_DURATION_SEC']),
            'speed_safety': CircularBuffer(self.config['SAFETY_WINDOW_DURATION_SEC']),
            'vsa_tcs_act': CircularBuffer(self.config['SAFETY_WINDOW_DURATION_SEC']),
            'abs_ebd_act': CircularBuffer(self.config['SAFETY_WINDOW_DURATION_SEC']),
        }
        
        # --- Persistent State for Scoring ---
        self.current_safety_window_penalty_sum = 0.0
        self.last_safety_window_reset_time = 0.0

        self.last_eco_score_calc_time = 0.0
        self.last_safety_score_calc_time = 0.0

        # --- Cooldown Timers for Events ---
        self.last_hard_accel_event_time = 0.0
        self.last_hard_brake_event_time = 0.0
        self.last_aggressive_corner_event_time = 0.0
        self.last_jerky_steering_event_time = 0.0
        self.last_vsa_abs_act_event_time = 0.0

        # --- Logging Setup ---
        self.log_file_path = log_file_path
        self.log_file = open(self.log_file_path, 'w') # 'w' will overwrite, 'a' will append
        self._log_message("--- Driving Event Log Started ---", to_console=True)


    def _log_message(self, message, current_timestamp=None, to_console=False):
        """Writes a timestamped message to the log file and optionally to console."""
        timestamp_str = f"[{current_timestamp:.3f}s] " if current_timestamp is not None else ""
        full_message = f"{timestamp_str}{message}\n"
        self.log_file.write(full_message)
        if to_console:
            print(message)

    def close_log(self):
        """Closes the log file."""
        if self.log_file and not self.log_file.closed:
            self._log_message("--- Driving Event Log Ended ---", to_console=True)
            self.log_file.close()

    def _get_values_in_window(self, buffer, duration_sec, current_timestamp):
        """Helper to get items within a time window, assumes buffer is trimmed externally."""
        return buffer.get_all_items()

    def _calculate_std_dev(self, values):
        if len(values) < 2:
            return 0.0
        mean = sum(values)/len(values)
        return math.sqrt(sum((x - mean)**2 for x in values) / (len(values) -1) if len(values) > 1 else len(values)) # Use N-1 for sample std dev

    def _calculate_average(self, values):
        if not values:
            return 0.0
        return sum(values) / len(values)
    
    def _send_event_notification(self, timestamp, eco_score, safety_score, reminder):
        
        return 0
    def process_can_data(self, new_can_data_packet):
        current_timestamp = new_can_data_packet.timestamp

        # Update all historical data buffers
        self._update_window_buffers(new_can_data_packet, current_timestamp)

        eco_score = None
        safety_score = None

        # Calculate Eco-Friendly Score (periodically)
        if (current_timestamp - self.last_eco_score_calc_time >= self.config['ECO_CALC_INTERVAL_SEC']):
            eco_score = self._calculate_eco_score(current_timestamp)
            self.last_eco_score_calc_time = current_timestamp

        # Calculate Safety Score (periodically)
        if (current_timestamp - self.last_safety_score_calc_time >= self.config['SAFETY_CALC_INTERVAL_SEC']):
            safety_score = self._calculate_safety_score(current_timestamp)
            self.last_safety_score_calc_time = current_timestamp
            
        return eco_score, safety_score

    def _update_window_buffers(self, packet, current_timestamp):
        # Add new data
        self.eco_buffers['trq_req'].add((current_timestamp, packet.ENG_DRIVER_REQ_TRQ_13C))
        self.eco_buffers['pedal_pos'].add((current_timestamp, packet.ENG_SMART_ACCELE_PEDAL_POS_13C))
        self.eco_buffers['speed'].add((current_timestamp, packet.VSA_ABS_FL_WHEEL_SPEED))
        self.eco_buffers['rpm'].add((current_timestamp, packet.ENG_ENG_SPEED))
        self.eco_buffers['gear'].add((current_timestamp, packet.CVT_GEAR_POSITION_IND_CVT))
        self.eco_buffers['is_progress'].add((current_timestamp, packet.ENG_IS_PROGRESS))

        self.safety_buffers['lon_g'].add((current_timestamp, packet.VSA_LON_G))
        self.safety_buffers['lat_g'].add((current_timestamp, packet.VSA_LAT_G))
        self.safety_buffers['yaw'].add((current_timestamp, packet.VSA_YAW_1))
        self.safety_buffers['steering_angle'].add((current_timestamp, packet.STR_ANGLE))
        self.safety_buffers['speed_safety'].add((current_timestamp, packet.VSA_ABS_FL_WHEEL_SPEED))
        self.safety_buffers['vsa_tcs_act'].add((current_timestamp, packet.VSA_VSA_TCS_ACT))
        self.safety_buffers['abs_ebd_act'].add((current_timestamp, packet.VSA_ABS_EBD_ACT))

        # Trim old data from buffers
        oldest_eco_ts = current_timestamp - self.config['ECO_WINDOW_DURATION_SEC']
        for buffer in self.eco_buffers.values():
            buffer.trim_older_than(oldest_eco_ts)

        oldest_safety_ts = current_timestamp - self.config['SAFETY_WINDOW_DURATION_SEC']
        for buffer in self.safety_buffers.values():
            buffer.trim_older_than(oldest_safety_ts)

    def _calculate_eco_score(self, current_timestamp):
        # Retrieve current window data
        trq_req_data = self.eco_buffers['trq_req'].get_all_items()
        pedal_pos_data = self.eco_buffers['pedal_pos'].get_all_items()
        speed_data = self.eco_buffers['speed'].get_all_items()
        rpm_data = self.eco_buffers['rpm'].get_all_items()
        gear_data = self.eco_buffers['gear'].get_all_items()
        is_progress_data = self.eco_buffers['is_progress'].get_all_items()

        s_accel = self._calculate_accel_smoothness_score(trq_req_data, pedal_pos_data, speed_data, current_timestamp)
        s_rpm = self._calculate_rpm_efficiency_score(rpm_data, speed_data, gear_data, current_timestamp)
        s_idle = self._calculate_idling_score(speed_data, rpm_data, is_progress_data, current_timestamp)

        score_eco = (self.config['W_accel_overall'] * s_accel +
                     self.config['W_rpm_overall'] * s_rpm +
                     self.config['W_idle_overall'] * s_idle)
        
        self._log_message(f"ECO Sub-Scores: Accel Smoothness={s_accel:.2f}, RPM Efficiency={s_rpm:.2f}, Idling={s_idle:.2f}", current_timestamp)
        return score_eco

    def _calculate_accel_smoothness_score(self, trq_req_data, pedal_pos_data, speed_data, current_timestamp):
        trq_req_deltas = []
        for i in range(1, len(trq_req_data)):
            delta_t = trq_req_data[i][0] - trq_req_data[i-1][0]
            if delta_t > 0:
                trq_req_deltas.append( (trq_req_data[i][1] - trq_req_data[i-1][1]) / delta_t )

        pedal_pos_deltas = []
        for i in range(1, len(pedal_pos_data)):
            delta_t = pedal_pos_data[i][0] - pedal_pos_data[i-1][0]
            if delta_t > 0:
                pedal_pos_deltas.append( (pedal_pos_data[i][1] - pedal_pos_data[i-1][1]) / delta_t )

        JT = self._calculate_std_dev(trq_req_deltas)
        JP = self._calculate_std_dev(pedal_pos_deltas)
        VS = self._calculate_std_dev(self.eco_buffers['speed'].get_values_only())

        E_agg = 0
        aggressive_accel_detected = False
        for i in range(1, len(speed_data)):
            delta_t = speed_data[i][0] - speed_data[i-1][0]
            if delta_t > 0:
                accel = (speed_data[i][1] - speed_data[i-1][1]) / delta_t # km/h/s
                if abs(accel) > self.config['AGGRESSIVE_ACCEL_DECEL_THRESHOLD_KMPHPS']:
                    E_agg += 1
                    aggressive_accel_detected = True
        
        score_JT = max(0, 1 - self.config['k1_accel_jt'] * JT)
        score_JP = max(0, 1 - self.config['k2_accel_jp'] * JP)
        score_VS = max(0, 1 - self.config['k3_accel_vs'] * VS)
        score_E_agg = max(0, 1 - self.config['k4_accel_eagg'] * E_agg)

        S_accel = (self.config['w1_accel'] * score_JT +
                   self.config['w2_accel'] * score_JP +
                   self.config['w3_accel'] * score_VS +
                   self.config['w4_accel'] * score_E_agg) * 100

        if aggressive_accel_detected:
            self._log_message(f"ECO: Aggressive Accel/Decel event detected (E_agg count: {E_agg})", current_timestamp)
        if JT > 0.5: # Example threshold for high jerk, adjust as needed
             self._log_message(f"ECO: High Torque Request Jerk (JT: {JT:.2f})", current_timestamp)
        if JP > 0.5: # Example threshold for high jerk, adjust as needed
             self._log_message(f"ECO: High Pedal Position Jerk (JP: {JP:.2f})", current_timestamp)
        if VS > 5.0: # Example threshold for high speed variation
             self._log_message(f"ECO: High Speed Variation (VS: {VS:.2f})", current_timestamp)

        return S_accel

    def _calculate_rpm_efficiency_score(self, rpm_data, speed_data, gear_data, current_timestamp):
        rpm_speed_ratios = []
        min_len = min(len(rpm_data), len(speed_data))
        for i in range(min_len):
            current_rpm = rpm_data[i][1]
            current_speed = speed_data[i][1]
            if current_speed > self.config['MIN_SPEED_FOR_RPM_RATIO_CALC']:
                rpm_speed_ratios.append(current_rpm / current_speed)

        R_ratio = self._calculate_average(rpm_speed_ratios)

        D_high_rpm = 0.0
        total_duration_in_window = 0.0
        for i in range(1, len(rpm_data)):
            current_rpm = rpm_data[i][1]
            delta_t = rpm_data[i][0] - rpm_data[i-1][0]
            if delta_t > 0:
                total_duration_in_window += delta_t
                if current_rpm > self.config['HIGH_RPM_THRESHOLD']:
                    D_high_rpm += delta_t
            
        D_high_rpm_proportion = D_high_rpm / total_duration_in_window if total_duration_in_window > 0 else 0.0

        score_R_ratio = max(0, 1 - self.config['k5_rpm_ratio'] * R_ratio)
        score_D_high_rpm = max(0, 1 - self.config['k6_rpm_high'] * D_high_rpm_proportion)

        S_rpm = (self.config['w5_rpm'] * score_R_ratio +
                 self.config['w6_rpm'] * score_D_high_rpm) * 100
        
        if D_high_rpm_proportion > 0.05: # If more than 5% of time is high RPM
            self._log_message(f"ECO: Inefficient High RPM driving detected (Proportion: {D_high_rpm_proportion:.2f})", current_timestamp)
        if R_ratio > 40: # Example threshold for high RPM/speed ratio
             self._log_message(f"ECO: High RPM/Speed Ratio (R_ratio: {R_ratio:.2f})", current_timestamp)

        return S_rpm

    def _calculate_idling_score(self, speed_data, rpm_data, is_progress_data, current_timestamp):
        T_idle = 0.0
        N_is_active = 0
        
        # Track idle-stop activations
        for i in range(1, len(is_progress_data)):
            if is_progress_data[i][1] == True and is_progress_data[i-1][1] == False:
                N_is_active += 1

        # Calculate idle time
        total_duration_in_window = 0.0
        min_len = min(len(speed_data), len(rpm_data))
        for i in range(1, min_len):
            current_speed = speed_data[i][1]
            current_rpm = rpm_data[i][1]
            delta_t = speed_data[i][0] - speed_data[i-1][0]
            if delta_t > 0:
                total_duration_in_window += delta_t
                if current_speed < self.config['MIN_DRIVING_SPEED_FOR_IDLE'] and \
                   current_rpm > self.config['MIN_IDLE_RPM_THRESHOLD']:
                    T_idle += delta_t
            
        T_idle_proportion = T_idle / total_duration_in_window if total_duration_in_window > 0 else 0.0

        score_T_idle = max(0, 1 - self.config['k7_idle_time'] * T_idle_proportion)
        score_IS = min(1, self.config['k8_idle_is'] * N_is_active)

        S_idle = (self.config['w7_idle'] * score_T_idle +
                  self.config['w8_idle'] * score_IS) * 100
        
        if T_idle_proportion > 0.1: # If more than 10% of window is inefficient idling
            self._log_message(f"ECO: Inefficient Idling detected (Proportion: {T_idle_proportion:.2f})", current_timestamp)
        # We can also log if idle-stop was *not* active during a stop, but that requires more state logic.
        # For now, just logging N_is_active provides insight.
        if N_is_active > 0:
            self._log_message(f"ECO: Idle-Stop activated (Count: {N_is_active})", current_timestamp)

        return S_idle

    def _calculate_safety_score(self, current_timestamp):
        # Reset penalty sum if a new safety window period has started
        if (current_timestamp - self.last_safety_window_reset_time >= self.config['SAFETY_WINDOW_DURATION_SEC']):
            self.current_safety_window_penalty_sum = 0.0
            self.last_safety_window_reset_time = current_timestamp
            # Also reset event cooldowns for a new clean window
            self.last_hard_accel_event_time = 0.0
            self.last_hard_brake_event_time = 0.0
            self.last_aggressive_corner_event_time = 0.0
            self.last_jerky_steering_event_time = 0.0
            self.last_vsa_abs_act_event_time = 0.0
            self._log_message(f"Safety Window Reset. New window started.", current_timestamp)


        # Get the most recent data point for immediate event checks
        last_lon_g_data = self.safety_buffers['lon_g'].get_last_value()
        last_lat_g_data = self.safety_buffers['lat_g'].get_last_value()
        last_yaw_data = self.safety_buffers['yaw'].get_last_value()
        last_steering_angle_data = self.safety_buffers['steering_angle'].get_last_value()
        last_speed_data = self.safety_buffers['speed_safety'].get_last_value()
        last_vsa_tcs_act_data = self.safety_buffers['vsa_tcs_act'].get_last_value()
        last_abs_ebd_act_data = self.safety_buffers['abs_ebd_act'].get_last_value()

        # Only proceed if we have enough data in the buffers
        if not all([last_lon_g_data, last_lat_g_data, last_yaw_data, last_steering_angle_data, last_speed_data]):
            return 100.0 # Return a perfect score if no data to evaluate unsafe acts

        # Get relevant recent history for rate-of-change or duration checks
        # These are used within the detect functions, no need to pass them explicitly here if functions fetch from self.buffers
        
        # Detect events and accumulate penalties
        self.current_safety_window_penalty_sum += \
            self._detect_hard_long_g_event(last_lon_g_data, current_timestamp)

        self.current_safety_window_penalty_sum += \
            self._detect_aggressive_cornering_event(last_lat_g_data, last_yaw_data, current_timestamp)

        self.current_safety_window_penalty_sum += \
            self._detect_jerky_steering_event(last_steering_angle_data, current_timestamp)

        self.current_safety_window_penalty_sum += \
            self._detect_system_intervention_event(last_vsa_tcs_act_data, last_abs_ebd_act_data, current_timestamp)

        # Overall Safety Score
        score_safety = max(0, 100 - self.current_safety_window_penalty_sum)
        return score_safety

    def _detect_hard_long_g_event(self, current_lon_g_data, current_timestamp):
        penalty = 0.0
        current_g = current_lon_g_data[1]
        
        # We need recent speed to ensure the vehicle is moving for G-force events
        recent_speed_history = self.safety_buffers['speed_safety'].get_all_items()
        current_speed = recent_speed_history[-1][1] if recent_speed_history else 0.0
        
        if current_speed < self.config['MIN_DRIVING_SPEED_FOR_IDLE']: # Don't penalize G-force if vehicle is stopped
            return 0.0

        # Check for hard acceleration
        if current_g > self.config['G_ACCEL_THRESHOLD_STATIC'] and \
           (current_timestamp - self.last_hard_accel_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            if self.config['G_MAX_EXPECTED_ACCEL'] > 0:
                penalty_factor = (current_g - self.config['G_ACCEL_THRESHOLD_STATIC']) / self.config['G_MAX_EXPECTED_ACCEL']
                if penalty_factor > 0:
                    new_penalty = self.config['C_LonG'] * (penalty_factor ** self.config['ALPHA_LON_G'])
                    penalty += new_penalty
                    self._log_message(f"SAFETY: Hard Accel detected! G={current_g:.2f}, Penalty={new_penalty:.2f}", current_timestamp)
            self.last_hard_accel_event_time = current_timestamp

        # Check for hard braking
        if current_g < -self.config['G_BRAKE_THRESHOLD_STATIC'] and \
           (current_timestamp - self.last_hard_brake_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            if self.config['G_MAX_EXPECTED_BRAKE'] > 0:
                penalty_factor = (abs(current_g) - self.config['G_BRAKE_THRESHOLD_STATIC']) / self.config['G_MAX_EXPECTED_BRAKE']
                if penalty_factor > 0:
                    new_penalty = self.config['C_LonG'] * (penalty_factor ** self.config['ALPHA_LON_G'])
                    penalty += new_penalty
                    self._log_message(f"SAFETY: Hard Brake detected! G={current_g:.2f}, Penalty={new_penalty:.2f}", current_timestamp)
            self.last_hard_brake_event_time = current_timestamp

        return penalty

    def _detect_aggressive_cornering_event(self, current_lat_g_data, current_yaw_data, current_timestamp):
        penalty = 0.0
        current_lat_g = current_lat_g_data[1]
        current_yaw = current_yaw_data[1]
        
        recent_speed_history = self.safety_buffers['speed_safety'].get_all_items()
        current_speed = recent_speed_history[-1][1] if recent_speed_history else 0.0

        if current_speed < self.config['MIN_DRIVING_SPEED_FOR_IDLE']: # Don't penalize cornering if vehicle is stopped
            return 0.0

        # Determine speed-dependent thresholds (linear interpolation)
        speed_ratio = max(0, min(1, (current_speed - 0) / (100 - 0))) # Normalize speed to 0-1 for interpolation
        lat_g_threshold = self.config['LAT_G_THRESHOLD_DYNAMIC_LOW_SPEED'] + \
                              (self.config['LAT_G_THRESHOLD_DYNAMIC_HIGH_SPEED'] - self.config['LAT_G_THRESHOLD_DYNAMIC_LOW_SPEED']) * speed_ratio
        yaw_threshold = self.config['YAW_THRESHOLD_DYNAMIC_LOW_SPEED'] + \
                        (self.config['YAW_THRESHOLD_DYNAMIC_HIGH_SPEED'] - self.config['YAW_THRESHOLD_DYNAMIC_LOW_SPEED']) * speed_ratio
            
        # Ensure thresholds are within reasonable bounds
        lat_g_threshold = max(0.5, min(self.config['MAX_LAT_G'], lat_g_threshold))
        yaw_threshold = max(5, min(self.config['MAX_YAW_RATE'], yaw_threshold))


        if (abs(current_lat_g) > lat_g_threshold or abs(current_yaw) > yaw_threshold) and \
           (current_timestamp - self.last_aggressive_corner_event_time >= self.config['EVENT_COOLDOWN_SEC']):

            # Calculate severity factor
            severity_lat_g = max(0, (abs(current_lat_g) - lat_g_threshold) / (self.config['MAX_LAT_G'] - lat_g_threshold)) if (self.config['MAX_LAT_G'] - lat_g_threshold) > 0 else 0
            severity_yaw = max(0, (abs(current_yaw) - yaw_threshold) / (self.config['MAX_YAW_RATE'] - yaw_threshold)) if (self.config['MAX_YAW_RATE'] - yaw_threshold) > 0 else 0

            severity_factor = max(severity_lat_g, severity_yaw) # Or average, or weighted sum

            if severity_factor > 0:
                new_penalty = self.config['C_LatG_Yaw'] * (severity_factor ** self.config['BETA_LAT_G_YAW'])
                penalty += new_penalty
                self._log_message(f"SAFETY: Aggressive Cornering detected! LatG={current_lat_g:.2f}, Yaw={current_yaw:.2f}, Penalty={new_penalty:.2f}", current_timestamp)
                self.last_aggressive_corner_event_time = current_timestamp

        return penalty

    def _detect_jerky_steering_event(self, current_steering_angle_data, current_timestamp):
        penalty = 0.0
        recent_steering_history = self.safety_buffers['steering_angle'].get_all_items()

        # Need at least two points to calculate rate of change accurately
        if len(recent_steering_history) < 2:
            return 0.0

        angle_prev_ts, angle_prev = recent_steering_history[-2]
        angle_curr_ts, angle_curr = recent_steering_history[-1]

        delta_t = angle_curr_ts - angle_prev_ts
        if delta_t > 0:
            steering_rate = abs((angle_curr - angle_prev) / delta_t) # degrees per second

            if steering_rate > self.config['STEERING_RATE_THRESHOLD'] and \
               (current_timestamp - self.last_jerky_steering_event_time >= self.config['EVENT_COOLDOWN_SEC']):
                
                if (self.config['MAX_STEERING_RATE'] - self.config['STEERING_RATE_THRESHOLD']) > 0:
                    penalty_factor = (steering_rate - self.config['STEERING_RATE_THRESHOLD']) / \
                                     (self.config['MAX_STEERING_RATE'] - self.config['STEERING_RATE_THRESHOLD'])
                    new_penalty = self.config['C_Steering'] * (penalty_factor ** self.config['GAMMA_STEERING'])
                    penalty += new_penalty
                    self._log_message(f"SAFETY: Jerky Steering detected! Rate={steering_rate:.2f} deg/s, Penalty={new_penalty:.2f}", current_timestamp)
                self.last_jerky_steering_event_time = current_timestamp

        return penalty

    def _detect_system_intervention_event(self, current_vsa_tcs_act_data, current_abs_ebd_act_data, current_timestamp):
        penalty = 0.0
        
        # Check for VSA/TCS activation
        if current_vsa_tcs_act_data and current_vsa_tcs_act_data[1] == True and \
           (current_timestamp - self.last_vsa_abs_act_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            new_penalty = self.config['C_Intervention']
            penalty += new_penalty
            self._log_message(f"SAFETY: VSA/TCS Activation detected! Penalty={new_penalty:.2f}", current_timestamp)
            self.last_vsa_abs_act_event_time = current_timestamp # Update cooldown

        # Check for ABS/EBD activation
        if current_abs_ebd_act_data and current_abs_ebd_act_data[1] == True and \
           (current_timestamp - self.last_vsa_abs_act_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            new_penalty = self.config['C_Intervention']
            penalty += new_penalty
            self._log_message(f"SAFETY: ABS/EBD Activation detected! Penalty={new_penalty:.2f}", current_timestamp)
            self.last_vsa_abs_act_event_time = current_timestamp

        return penalty