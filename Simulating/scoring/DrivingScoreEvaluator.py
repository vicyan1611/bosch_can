import math
import time # For simulation of timestamps
from scoring.CircularBuffer import CircularBuffer

class DrivingScoreEvaluator:
    def __init__(self, config=None, log_file_path='driving_events_log.txt'):
        # --- Configuration (Tunable Parameters) ---
        self.config = {
            # Window Durations (seconds)
            'ECO_WINDOW_DURATION_SEC': 60,
            'SAFETY_WINDOW_DURATION_SEC': 10,
            'EVENT_CHECK_HISTORY_SEC': 1, # For rate-of-change/duration checks

            # Calculation Intervals (seconds)
            'ECO_CALC_INTERVAL_SEC': 3.0,
            'SAFETY_CALC_INTERVAL_SEC': 3.0,

            # Eco-Friendly Thresholds & Weights
            'MIN_SPEED_FOR_RPM_RATIO_CALC': 5.0, # km/h
            'HIGH_RPM_THRESHOLD': 2500, # rpm
            'MIN_DRIVING_SPEED_FOR_IDLE': 0.5, # km/h
            'MIN_IDLE_RPM_THRESHOLD': 300, # rpm
            'AGGRESSIVE_ACCEL_DECEL_THRESHOLD_KMPHPS': 5.0, # km/h/s
            'GEAR_CHANGE_THRESHOLD': 3,

            'k1_accel_jt': 0.01,  'k2_accel_jp': 0.05,  'k3_accel_vs': 0.001, 'k4_accel_eagg': 0.5,
            'w1_accel': 0.25, 'w2_accel': 0.25, 'w3_accel': 0.25, 'w4_accel': 0.25,
            
            'k5_rpm_ratio': 0.001, 'k6_rpm_high': 0.1,
            'w5_rpm': 0.7, 'w6_rpm': 0.3,

            'k7_idle_time': 0.2, 'k8_idle_is': 1.0,
            'w7_idle': 0.9, 'w8_idle': 0.1,

            'W_accel_overall': 0.4, 'W_rpm_overall': 0.2, 'W_idle_overall': 0.3, 'W_gear_overall': 0.1,

            # Safety Thresholds & Weights
            'G_ACCEL_THRESHOLD_STATIC': 3.9, # m/s^2 (approx 0.4G)
            'G_BRAKE_THRESHOLD_STATIC': 5.8, # m/s^2 (approx 0.6G)
            'G_MAX_EXPECTED_ACCEL': 8.0, # m/s^2 (approx 1.0G, for normalization)
            'G_MAX_EXPECTED_BRAKE': 10.0, # m/s^2

            'LAT_G_THRESHOLD_DYNAMIC_LOW_SPEED': 2.9, # m/s^2 (e.g., at 0-20 km/h)
            'LAT_G_THRESHOLD_DYNAMIC_HIGH_SPEED': 4.9, # m/s^2 (e.g., at >80 km/h)
            'YAW_THRESHOLD_DYNAMIC_LOW_SPEED': 20, # deg/s
            'YAW_THRESHOLD_DYNAMIC_HIGH_SPEED': 40, # deg/s
            'MAX_LAT_G': 9.8, # m/s^2
            'MAX_YAW_RATE': 80, # deg/s

            'STEERING_RATE_THRESHOLD': 5, # deg/s
            'MAX_STEERING_RATE': 300, # deg/s (e.g., lock-to-lock in ~1 sec)

            'C_LonG': 25,
            'C_LatG_Yaw': 35,
            'C_Steering': 15,
            'C_Intervention': 70,

            'ALPHA_LON_G': -1.5,
            'BETA_LAT_G_YAW': -1.0,
            'GAMMA_STEERING': 1.0,
            
            'IDLING_COOLDOWN_SEC': 10.0,  # Idling event cooldown
            'SCORE_UPDATE_COOLDOWN_SEC': 1.0,  # Score update cooldown

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
        
        self.event_buffers = {
            'score_update': 0.0,
            'hard_accel': 0.0,
            'hard_brake': 0.0,
            'aggressive_cornering': 0.0,
            'safety_intervention': 0.0,
            'jerky_steering': 0.0,
            'vsa_abs_act': 0.0,
            'high_rpm': 0.0,
            'idling': 0.0,
            'gear_change': 0.0,
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

        self.safety_score = 100
        self.eco_score = 100
        
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
        return math.sqrt(sum((x - mean)**2 for x in values) / (len(values) -1))

    def _calculate_average(self, values):
        if not values:
            return 0.0
        return sum(values) / len(values)

    def _send_event(self, safety_score, eco_score, feedback):
        """Sends event data to the FastAPI backend."""
        import requests
        import json
        url = "http://127.0.0.1:8000/event"
        try:
            payload = {"safety_score": int(safety_score), "eco_score": int(eco_score), "reminder": feedback}
            requests.post(url, json=payload)
        except requests.exceptions.ConnectionError:
            print("Dashboard is not running. Could not send update.")

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

        should_send_event = False

        if eco_score is not None:
            self.eco_score = eco_score
            should_send_event = True
        if safety_score is not None:
            self.safety_score = safety_score
            should_send_event = True

        if should_send_event:
            self._send_event(self.safety_score, self.eco_score, "")

        self._log_message(f"Time: {current_timestamp:.1f}s | Eco Score: {self.eco_score:.2f} | Safety Score: {self.safety_score:.2f}")
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
        trq_req_data = self.eco_buffers['trq_req'].get_all_items()
        pedal_pos_data = self.eco_buffers['pedal_pos'].get_all_items()
        speed_data = self.eco_buffers['speed'].get_all_items()
        rpm_data = self.eco_buffers['rpm'].get_all_items()
        gear_data = self.eco_buffers['gear'].get_all_items()
        is_progress_data = self.eco_buffers['is_progress'].get_all_items()

        s_accel = self._calculate_accel_smoothness_score(trq_req_data, pedal_pos_data, speed_data, current_timestamp)
        s_rpm = self._calculate_rpm_efficiency_score(rpm_data, speed_data, gear_data, current_timestamp)
        s_idle = self._calculate_idling_score_alternative(speed_data, rpm_data, is_progress_data, current_timestamp)
        s_gear = self._calculate_gear_selection_score(gear_data, current_timestamp)

        score_eco = (self.config['W_accel_overall'] * s_accel +
                     self.config['W_rpm_overall'] * s_rpm +
                     self.config['W_idle_overall'] * s_idle +
                     self.config['W_gear_overall'] * s_gear)
        
        self._log_message(f"ECO Sub-Scores: Accel Smoothness={s_accel:.2f}, RPM Efficiency={s_rpm:.2f}, Idling={s_idle:.2f}, Gear Change={s_gear:.2f}", current_timestamp)
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
        
        speed_values = [d[1] for d in speed_data]
        VS = self._calculate_std_dev(speed_values)

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
        if JT > 0.5:
             self._log_message(f"ECO: High Torque Request Jerk (JT: {JT:.2f})", current_timestamp)
        if JP > 0.5:
             self._log_message(f"ECO: High Pedal Position Jerk (JP: {JP:.2f})", current_timestamp)
        if VS > 5.0:
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
        
        should_send_event = False
        if D_high_rpm_proportion > 0.05:
            self._log_message(f"ECO: Inefficient High RPM driving detected (Proportion: {D_high_rpm_proportion:.2f})", current_timestamp)
            should_send_event = True
        if R_ratio > 40:
            self._log_message(f"ECO: High RPM/Speed Ratio (R_ratio: {R_ratio:.2f})", current_timestamp)
            should_send_event = True
            
        if should_send_event: 
            self._send_event(self.safety_score, self.eco_score, "Inefficient High RPM driving detected")

        return S_rpm

    def _calculate_idling_score(self, speed_data, rpm_data, is_progress_data, current_timestamp):
        T_idle = 0.0
        N_is_active = 0
        
        for i in range(1, len(is_progress_data)):
            if is_progress_data[i][1] == True and is_progress_data[i-1][1] == False:
                N_is_active += 1

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
        
        if T_idle_proportion > 0.1:
            self._log_message(f"ECO: Inefficient Idling detected (Proportion: {T_idle_proportion:.2f})", current_timestamp)
            self._send_event(self.safety_score, self.eco_score, 'Inefficient Idling detected')
        if N_is_active > 0:
            self._log_message(f"ECO: Idle-Stop activated (Count: {N_is_active})", current_timestamp)

        return S_idle

    def _calculate_idling_score_alternative(self, speed_data, rpm_data, is_progress_data, current_timestamp):
        """
        Alternative approach: More granular detection of different idling patterns
        """
        
        # Different categories of idling behavior
        metrics = {
            'normal_idle_time': 0.0,        # Reasonable idle (800-1200 RPM)
            'high_idle_time': 0.0,          # Wasteful idle (1200-2000 RPM)  
            'excessive_idle_time': 0.0,     # Very wasteful (>2000 RPM)
            'engine_off_time': 0.0,         # Engine off while stationary (good!)
            'total_stationary_time': 0.0,   # Total time not moving
            'is_activations': 0,            # Idle-stop system uses
            'total_duration': 0.0
        }
        
        # Count idle-stop activations
        for i in range(1, len(is_progress_data)):
            if is_progress_data[i][1] == True and is_progress_data[i-1][1] == False:
                metrics['is_activations'] += 1
        
        # Analyze each time segment
        min_len = min(len(speed_data), len(rpm_data))
        for i in range(1, min_len):
            current_speed = speed_data[i][1]
            current_rpm = rpm_data[i][1]
            delta_t = speed_data[i][0] - speed_data[i-1][0]
            
            if delta_t > 0:
                metrics['total_duration'] += delta_t
                
                # Only analyze when vehicle is stationary
                if current_speed < self.config['MIN_DRIVING_SPEED_FOR_IDLE']:
                    metrics['total_stationary_time'] += delta_t
                    
                    # Categorize the type of idling
                    if current_rpm < 500:  # Engine likely off
                        metrics['engine_off_time'] += delta_t
                    elif current_rpm <= 1200:  # Normal idle range
                        metrics['normal_idle_time'] += delta_t
                    elif current_rpm <= 2000:  # High idle - wasteful
                        metrics['high_idle_time'] += delta_t
                    else:  # Excessive RPM - very wasteful
                        metrics['excessive_idle_time'] += delta_t
        
        # Calculate efficiency score based on idling behavior
        if metrics['total_stationary_time'] > 0:
            # Percentages of stationary time spent in each mode
            pct_engine_off = metrics['engine_off_time'] / metrics['total_stationary_time']
            pct_normal_idle = metrics['normal_idle_time'] / metrics['total_stationary_time']
            pct_high_idle = metrics['high_idle_time'] / metrics['total_stationary_time']
            pct_excessive_idle = metrics['excessive_idle_time'] / metrics['total_stationary_time']
            
            # Scoring: Reward engine-off time, neutral for normal idle, penalize high/excessive
            score_composition = (
                pct_engine_off * 1.0 +      # Full points for engine off
                pct_normal_idle * 0.7 +     # Reduced points for normal idle
                pct_high_idle * 0.3 +       # Low points for high idle
                pct_excessive_idle * 0.0    # No points for excessive idle
            )
            
            # Add bonus for idle-stop usage
            stationary_periods = metrics['total_stationary_time'] / 10  # Assume 10s per period
            if stationary_periods > 0:
                is_usage_bonus = min(0.2, metrics['is_activations'] / stationary_periods * 0.2)
                score_composition += is_usage_bonus
        else:
            score_composition = 1.0  # No stationary time = perfect score
        
        S_idle = max(0, min(100, score_composition * 100))
        
        # Detailed logging
        if metrics['total_stationary_time'] > 0:
            stationary_pct = metrics['total_stationary_time'] / metrics['total_duration'] * 100
            self._log_message(
                f"ECO Idling Analysis - Stationary: {stationary_pct:.1f}% | "
                f"Engine Off: {pct_engine_off:.1%}, Normal Idle: {pct_normal_idle:.1%}, "
                f"High Idle: {pct_high_idle:.1%}, Excessive: {pct_excessive_idle:.1%} | "
                f"IS Activations: {metrics['is_activations']}", 
                current_timestamp
            )
            
            # Generate specific feedback
            if time.time() - self.event_buffers['idling'] > self.config['IDLING_COOLDOWN_SEC']:
                if pct_excessive_idle > 0.1:  # >10% excessive idling
                    self._send_event(self.safety_score, self.eco_score, 
                                'Excessive engine revving while stationary')
                elif pct_high_idle > 0.3:  # >30% high idling  
                    self._send_event(self.safety_score, self.eco_score,
                                'High RPM idling detected - consider idle-stop')
                elif pct_normal_idle > 0.8 and metrics['is_activations'] == 0:
                    self._send_event(self.safety_score, self.eco_score,
                                'Consider using idle-stop system to save fuel')
                self.event_buffers['idling'] = time.time()
        
        return S_idle

    def _calculate_gear_selection_score(self, gear_data, current_timestamp):
        # Count gear changes
        gear_changes = 0
        for i in range(1, len(gear_data)):
            prev_gear = gear_data[i-1][1]
            curr_gear = gear_data[i][1]
            if curr_gear != prev_gear:
                gear_changes += 1

        # Threshold for excessive gear changes (tunable)
        threshold = self.config['GEAR_CHANGE_THRESHOLD']

        if gear_changes > threshold:
            self._log_message(
                f"ECO: Excessive gear changes detected (Count: {gear_changes})",
                current_timestamp
            )
            self._send_event(self.safety_score, self.eco_score, "Excessive gear changes detected")

        # Score calculation: penalize if above threshold
        penalty_factor = max(0, (gear_changes - threshold) / threshold)
        score = max(0, 100 - penalty_factor * 100)

        return score
    
    def _calculate_safety_score(self, current_timestamp):
        # --- SAFETY SCORE USES A MIX OF TYPES ---
        if (current_timestamp - self.last_safety_window_reset_time >= self.config['SAFETY_WINDOW_DURATION_SEC']):
            self.current_safety_window_penalty_sum = 0.0
            self.last_safety_window_reset_time = current_timestamp
            self.last_hard_accel_event_time = 0.0
            self.last_hard_brake_event_time = 0.0
            self.last_aggressive_corner_event_time = 0.0
            self.last_jerky_steering_event_time = 0.0
            self.last_vsa_abs_act_event_time = 0.0
            self._log_message(f"Safety Window Reset. New window started.", current_timestamp)

        last_lon_g_data = self.safety_buffers['lon_g'].get_last_value()
        last_lat_g_data = self.safety_buffers['lat_g'].get_last_value()
        last_yaw_data = self.safety_buffers['yaw'].get_last_value()
        last_steering_angle_data = self.safety_buffers['steering_angle'].get_last_value()
        last_speed_data = self.safety_buffers['speed_safety'].get_last_value()
        last_vsa_tcs_act_data = self.safety_buffers['vsa_tcs_act'].get_last_value()
        last_abs_ebd_act_data = self.safety_buffers['abs_ebd_act'].get_last_value()
        pedal_pos_data = self.eco_buffers['pedal_pos'].get_all_items()

        if not all([last_lon_g_data, last_lat_g_data, last_yaw_data, last_steering_angle_data, last_speed_data]):
            return 100.0

        hard_lon_g_pen = self._detect_hard_long_g_event_alternative(last_lon_g_data, current_timestamp)
        agg_corner_pen = self._detect_aggressive_cornering_event(last_lat_g_data, last_yaw_data, current_timestamp)
        jerky_pen = self._detect_jerky_steering_event_alternative(last_steering_angle_data, current_timestamp)
        sys_inter_pen = self._detect_system_intervention_event(last_vsa_tcs_act_data, last_abs_ebd_act_data, current_timestamp)
        pedal_pen = self._detect_pedal_press(pedal_pos_data, current_timestamp)

        self.current_safety_window_penalty_sum = 0
        self.current_safety_window_penalty_sum += hard_lon_g_pen
        self.current_safety_window_penalty_sum += agg_corner_pen
        self.current_safety_window_penalty_sum += jerky_pen
        self.current_safety_window_penalty_sum += sys_inter_pen
        self.current_safety_window_penalty_sum += pedal_pen
            
        score_safety = max(0, 100 - self.current_safety_window_penalty_sum)
        self._log_message(f"Safety Sub-Scores: Hard Long G={hard_lon_g_pen:.2f}, Aggressive Cornering={agg_corner_pen:.2f}, Jerky Steering={jerky_pen:.2f}, System Intervention={sys_inter_pen:.2f}, Pedal={pedal_pen:.2f}, penalty={self.current_safety_window_penalty_sum:.2f}", current_timestamp)
        return score_safety

    def _detect_hard_long_g_event(self, current_lon_g_data, current_timestamp):
        penalty = 0.0
        # CORRECTED: Use .value for this complex signal
        current_g = float(current_lon_g_data[1])
        
        recent_speed_history = self.safety_buffers['speed_safety'].get_all_items()
        # CORRECTED: Do NOT use .value for this primitive signal
        current_speed = recent_speed_history[-1][1] if recent_speed_history else 0.0
        
        if current_speed < self.config['MIN_DRIVING_SPEED_FOR_IDLE']:
            return 0.0

        if current_g > self.config['G_ACCEL_THRESHOLD_STATIC'] and \
           (current_timestamp - self.last_hard_accel_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            if self.config['G_MAX_EXPECTED_ACCEL'] > 0:
                penalty_factor = (current_g - self.config['G_ACCEL_THRESHOLD_STATIC']) / self.config['G_MAX_EXPECTED_ACCEL']
                if penalty_factor > 0:
                    new_penalty = self.config['C_LonG'] * (penalty_factor ** self.config['ALPHA_LON_G'])
                    penalty += new_penalty
                    self._log_message(f"SAFETY: Hard Accel detected! G={current_g:.2f}, Penalty={new_penalty:.2f}", current_timestamp)
                    self._send_event(self.safety_score, self.eco_score, "Hard Accelleration detected")
            self.last_hard_accel_event_time = current_timestamp

        if current_g < -self.config['G_BRAKE_THRESHOLD_STATIC'] and \
           (current_timestamp - self.last_hard_brake_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            if self.config['G_MAX_EXPECTED_BRAKE'] > 0:
                penalty_factor = (abs(current_g) - self.config['G_BRAKE_THRESHOLD_STATIC']) / self.config['G_MAX_EXPECTED_BRAKE']
                if penalty_factor > 0:
                    new_penalty = self.config['C_LonG'] * (penalty_factor ** self.config['ALPHA_LON_G'])
                    penalty += new_penalty
                    self._log_message(f"SAFETY: Hard Brake detected! G={current_g:.2f}, Penalty={new_penalty:.2f}", current_timestamp)
                    self._send_event(self.safety_score, self.eco_score, "Hard Brake detected")

            self.last_hard_brake_event_time = current_timestamp

        return penalty

    def _detect_pedal_press(self, pedal_pos_data, current_timestamp):
        """
        Detects aggressive acceleration based on pedal position patterns.
        Analyzes pedal press rate, sustained high pedal positions, and sudden changes.
        """
        penalty = 0.0
        
        if len(pedal_pos_data) < 2:
            return 0.0
        
        # Extract pedal positions and timestamps
        pedal_positions = [data[1] for data in pedal_pos_data]
        timestamps = [data[0] for data in pedal_pos_data]
        
        # Calculate current pedal position and average
        current_pedal_pos = pedal_positions[-1]
        avg_pedal_pos = sum(pedal_positions) / len(pedal_positions)
        max_pedal_pos = max(pedal_positions)
        
        # Calculate pedal press rates (change per second)
        pedal_rates = []
        for i in range(1, len(pedal_pos_data)):
            dt = timestamps[i] - timestamps[i-1]
            if dt > 0:
                rate = (pedal_positions[i] - pedal_positions[i-1]) / dt
                pedal_rates.append(rate)
        
        # Calculate metrics for aggressive detection
        max_press_rate = max(pedal_rates) if pedal_rates else 0
        avg_press_rate = sum([r for r in pedal_rates if r > 0]) / max(1, len([r for r in pedal_rates if r > 0]))
        
        # Calculate standard deviation of pedal position (indicates erratic behavior)
        pedal_std_dev = 0
        if len(pedal_positions) > 1:
            mean_pos = sum(pedal_positions) / len(pedal_positions)
            variance = sum((pos - mean_pos) ** 2 for pos in pedal_positions) / len(pedal_positions)
            pedal_std_dev = variance ** 0.5
        
        # Count sudden pedal changes (aggressive inputs)
        sudden_changes = 0
        large_changes = 0
        time_window = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 1.0
        
        SUDDEN_CHANGE_THRESHOLD = 15.0  # % change per second
        LARGE_CHANGE_THRESHOLD = 25.0   # % absolute change
        
        for i in range(1, len(pedal_pos_data)):
            dt = timestamps[i] - timestamps[i-1]
            if dt > 0:
                change_rate = abs(pedal_positions[i] - pedal_positions[i-1]) / dt
                change_magnitude = abs(pedal_positions[i] - pedal_positions[i-1])
                
                if change_rate > SUDDEN_CHANGE_THRESHOLD:
                    sudden_changes += 1
                if change_magnitude > LARGE_CHANGE_THRESHOLD:
                    large_changes += 1
        
        # Calculate frequencies
        sudden_change_frequency = sudden_changes / time_window if time_window > 0 else 0
        large_change_frequency = large_changes / time_window if time_window > 0 else 0
        
        # Calculate time spent at high pedal positions
        high_pedal_time = 0.0
        extreme_pedal_time = 0.0
        total_time = time_window
        
        HIGH_PEDAL_THRESHOLD = 50.0    # % - sustained high acceleration
        EXTREME_PEDAL_THRESHOLD = 70.0 # % - extreme acceleration
        
        for i in range(1, len(pedal_pos_data)):
            dt = timestamps[i] - timestamps[i-1]
            if dt > 0:
                if pedal_positions[i] > EXTREME_PEDAL_THRESHOLD:
                    extreme_pedal_time += dt
                elif pedal_positions[i] > HIGH_PEDAL_THRESHOLD:
                    high_pedal_time += dt
        
        high_pedal_proportion = high_pedal_time / total_time if total_time > 0 else 0
        extreme_pedal_proportion = extreme_pedal_time / total_time if total_time > 0 else 0
        
        # Detection logic with tiered penalties
        aggressive_detected = False
        penalty_reason = ""
        detection_method = ""
        
        # Method 1: Extremely rapid pedal press (slam acceleration)
        if max_press_rate > 70:  # >80% per second change
            penalty += min(35, max_press_rate * 0.3)
            aggressive_detected = True
            penalty_reason = f"Extreme pedal slam: {max_press_rate:.1f}%/s"
            detection_method = "Pedal_Slam"
            
        # Method 2: Sustained extreme pedal position
        elif extreme_pedal_proportion > 0.3:  # >30% of time at >90% pedal
            penalty += min(30, extreme_pedal_proportion * 80)
            aggressive_detected = True
            penalty_reason = f"Sustained extreme acceleration: {extreme_pedal_proportion:.1%} at >{EXTREME_PEDAL_THRESHOLD}%"
            detection_method = "Sustained_Extreme"
            
        # Method 3: High frequency of sudden pedal changes
        elif sudden_change_frequency > 1.5:  # More than 1.5 sudden changes per second
            penalty += min(25, sudden_change_frequency * 15)
            aggressive_detected = True
            penalty_reason = f"Frequent sudden pedal changes: {sudden_change_frequency:.2f}/s"
            detection_method = "Frequent_Changes"
            
        # Method 4: High pedal position with erratic behavior
        elif avg_pedal_pos > 75 and pedal_std_dev > 15:
            penalty += min(20, (avg_pedal_pos - 50) * 0.3 + pedal_std_dev * 0.5)
            aggressive_detected = True
            penalty_reason = f"Erratic high pedal: avg={avg_pedal_pos:.1f}%, std={pedal_std_dev:.1f}%"
            detection_method = "Erratic_High"
            
        # Method 5: High sustained pedal with rapid average press rate
        elif high_pedal_proportion > 0.5 and avg_press_rate > 30:
            penalty += min(20, high_pedal_proportion * 25 + avg_press_rate * 0.3)
            aggressive_detected = True
            penalty_reason = f"Sustained aggressive driving: {high_pedal_proportion:.1%} high pedal, avg rate {avg_press_rate:.1f}%/s"
            detection_method = "Sustained_Aggressive"
            
        # Method 6: Large pedal changes with high frequency
        elif large_change_frequency > 0.8 and max_pedal_pos > 80:
            penalty += min(15, large_change_frequency * 15)
            aggressive_detected = True
            penalty_reason = f"Frequent large pedal changes: {large_change_frequency:.2f}/s, max {max_pedal_pos:.1f}%"
            detection_method = "Large_Changes"
        
        # Apply penalty with cooldown (reuse existing event buffer)
        if aggressive_detected:
            # Check if we need to add a new cooldown timer for pedal events
            if not hasattr(self, 'last_aggressive_pedal_event_time'):
                self.last_aggressive_pedal_event_time = 0.0
                
            if (current_timestamp - self.last_aggressive_pedal_event_time >= self.config['EVENT_COOLDOWN_SEC']):
                self._log_message(
                    f"SAFETY: Aggressive Pedal Use detected ({detection_method})! {penalty_reason}, Penalty={penalty:.2f}",
                    current_timestamp
                )
                self._send_event(self.safety_score, self.eco_score, f"Aggressive acceleration detected: {penalty_reason}")
                self.last_aggressive_pedal_event_time = current_timestamp
            else:
                # Reset penalty if still in cooldown
                penalty = 0.0
                self._log_message(f"DEBUG: Aggressive pedal detected ({detection_method}) but in cooldown: {penalty_reason}", current_timestamp)
        
        # Debug logging for analysis
        self._log_message(
            f"DEBUG Pedal Analysis: "
            f"Current={current_pedal_pos:.1f}%, Avg={avg_pedal_pos:.1f}%, Max={max_pedal_pos:.1f}%, "
            f"Max_rate={max_press_rate:.1f}%/s, Avg_rate={avg_press_rate:.1f}%/s, "
            f"Std_dev={pedal_std_dev:.1f}%, Sudden_freq={sudden_change_frequency:.2f}/s, "
            f"High_time={high_pedal_proportion:.1%}, Extreme_time={extreme_pedal_proportion:.1%}",
            current_timestamp
        )
        
        return penalty
            
            
    def _detect_hard_long_g_event_alternative(self, current_lon_g_data, current_timestamp):
        """
        Alternative approach with tiered penalty system
        """
        penalty = 0.0
        current_g = float(current_lon_g_data[1])
        
        recent_speed_history = self.safety_buffers['speed_safety'].get_all_items()
        current_speed = recent_speed_history[-1][1] if recent_speed_history else 0.0
        
        if current_speed < self.config['MIN_DRIVING_SPEED_FOR_IDLE']:
            return 0.0

        # --- TIERED ACCELERATION PENALTIES ---
        if current_g > 0:  # Positive acceleration
            if current_g > 7.0:  # Extreme acceleration (>0.7G)
                penalty += 40
                event_type = "Extreme"
            elif current_g > 5.0:  # Hard acceleration (>0.5G)  
                penalty += 25
                event_type = "Hard"
            elif current_g > self.config['G_ACCEL_THRESHOLD_STATIC']:  # Moderate but notable
                penalty += 10
                event_type = "Moderate"
            else:
                event_type = None
                
            if event_type and (current_timestamp - self.last_hard_accel_event_time >= self.config['EVENT_COOLDOWN_SEC']):
                self._log_message(
                    f"SAFETY: {event_type} Acceleration! G={current_g:.2f}m/s², Penalty={penalty:.0f}",
                    current_timestamp
                )
                self._send_event(self.safety_score, self.eco_score, f"{event_type} Acceleration detected")
                self.last_hard_accel_event_time = current_timestamp

        # --- TIERED BRAKING PENALTIES ---
        elif current_g < 0:  # Negative acceleration (braking)
            abs_g = abs(current_g)
            if abs_g > 8.0:  # Extreme braking (>0.8G)
                penalty += 40
                event_type = "Extreme"
            elif abs_g > 6.0:  # Hard braking (>0.6G)
                penalty += 25  
                event_type = "Hard"
            elif abs_g > self.config['G_BRAKE_THRESHOLD_STATIC']:  # Moderate but notable
                penalty += 10
                event_type = "Moderate"
            else:
                event_type = None
                
            if event_type and (current_timestamp - self.last_hard_brake_event_time >= self.config['EVENT_COOLDOWN_SEC']):
                self._log_message(
                    f"SAFETY: {event_type} Braking! G={current_g:.2f}m/s², Penalty={penalty:.0f}",
                    current_timestamp
                )
                self._send_event(self.safety_score, self.eco_score, f"{event_type} Braking detected")
                self.last_hard_brake_event_time = current_timestamp

        return penalty

    def _detect_aggressive_cornering_event(self, current_lat_g_data, current_yaw_data, current_timestamp):
        penalty = 0.0
        current_lat_g = float(current_lat_g_data[1])
        current_yaw = float(current_yaw_data[1])
        
        recent_speed_history = self.safety_buffers['speed_safety'].get_all_items()
        current_speed = recent_speed_history[-1][1] if recent_speed_history else 0.0

        if current_speed < self.config['MIN_DRIVING_SPEED_FOR_IDLE']:
            return 0.0

        speed_ratio = max(0, min(1, (current_speed - 0) / (100 - 0)))
        lat_g_threshold = self.config['LAT_G_THRESHOLD_DYNAMIC_LOW_SPEED'] + \
                              (self.config['LAT_G_THRESHOLD_DYNAMIC_HIGH_SPEED'] - self.config['LAT_G_THRESHOLD_DYNAMIC_LOW_SPEED']) * speed_ratio
        yaw_threshold = self.config['YAW_THRESHOLD_DYNAMIC_LOW_SPEED'] + \
                        (self.config['YAW_THRESHOLD_DYNAMIC_HIGH_SPEED'] - self.config['YAW_THRESHOLD_DYNAMIC_LOW_SPEED']) * speed_ratio
            
        lat_g_threshold = max(0.5, min(self.config['MAX_LAT_G'], lat_g_threshold))
        yaw_threshold = max(5, min(self.config['MAX_YAW_RATE'], yaw_threshold))


        if (abs(current_lat_g) > lat_g_threshold or abs(current_yaw) > yaw_threshold) and \
           (current_timestamp - self.last_aggressive_corner_event_time >= self.config['EVENT_COOLDOWN_SEC']):

            severity_lat_g = max(0, (abs(current_lat_g) - lat_g_threshold) / (self.config['MAX_LAT_G'] - lat_g_threshold)) if (self.config['MAX_LAT_G'] - lat_g_threshold) > 0 else 0
            severity_yaw = max(0, (abs(current_yaw) - yaw_threshold) / (self.config['MAX_YAW_RATE'] - yaw_threshold)) if (self.config['MAX_YAW_RATE'] - yaw_threshold) > 0 else 0
            severity_factor = max(severity_lat_g, severity_yaw)

            if severity_factor > 0:
                new_penalty = self.config['C_LatG_Yaw'] * (severity_factor ** self.config['BETA_LAT_G_YAW'])
                penalty += new_penalty
                self._log_message(f"SAFETY: Aggressive Cornering detected! LatG={current_lat_g:.2f}, Yaw={current_yaw:.2f}, Penalty={new_penalty:.2f}", current_timestamp)
                self._send_event(self.safety_score, self.eco_score, "Aggressive Cornering detected")

                self.last_aggressive_corner_event_time = current_timestamp

        return penalty

    def _detect_jerky_steering_event(self, current_steering_angle_data, current_timestamp):
        penalty = 0.0
        recent_steering_history = self.safety_buffers['steering_angle'].get_all_items()

        if len(recent_steering_history) < 2:
            return 0.0

        angle_prev_ts, angle_prev_val = recent_steering_history[-2]
        angle_curr_ts, angle_curr_val = recent_steering_history[-1]
        
        angle_prev = float(angle_prev_val)
        angle_curr = float(angle_curr_val)

        delta_t = angle_curr_ts - angle_prev_ts
        if delta_t > 0:
            steering_rate = abs((angle_curr - angle_prev) / delta_t)

            if steering_rate > self.config['STEERING_RATE_THRESHOLD'] and \
               (current_timestamp - self.last_jerky_steering_event_time >= self.config['EVENT_COOLDOWN_SEC']):
                
                if (self.config['MAX_STEERING_RATE'] - self.config['STEERING_RATE_THRESHOLD']) > 0:
                    penalty_factor = (steering_rate - self.config['STEERING_RATE_THRESHOLD']) / \
                                     (self.config['MAX_STEERING_RATE'] - self.config['STEERING_RATE_THRESHOLD'])
                    new_penalty = self.config['C_Steering'] * (penalty_factor ** self.config['GAMMA_STEERING'])
                    penalty += new_penalty
                    self._log_message(f"SAFETY: Jerky Steering detected! Rate={steering_rate:.2f} deg/s, Penalty={new_penalty:.2f}", current_timestamp)
                    self._send_event(self.safety_score, self.eco_score, "Jerky Steering detected")

                self.last_jerky_steering_event_time = current_timestamp

        return penalty

    def _detect_jerky_steering_event_alternative(self, current_steering_angle_data, current_timestamp):
        """
        Improved oscillation-based jerky steering detection with noise filtering
        """
        penalty = 0.0
        recent_steering_history = self.safety_buffers['steering_angle'].get_all_items()

        # Need minimum samples for oscillation detection
        if len(recent_steering_history) < 6:
            self._log_message("DEBUG: Not enough steering data for oscillation detection", current_timestamp)
            return 0.0

        # Extract angles and apply noise filtering
        raw_angles = [float(item[1]) for item in recent_steering_history]
        timestamps = [item[0] for item in recent_steering_history]
        
        # Simple moving average filter to reduce noise (window size = 3)
        filtered_angles = []
        for i in range(len(raw_angles)):
            if i < 1:
                filtered_angles.append(raw_angles[i])
            elif i < 2:
                filtered_angles.append((raw_angles[i-1] + raw_angles[i]) / 2)
            else:
                filtered_angles.append((raw_angles[i-2] + raw_angles[i-1] + raw_angles[i]) / 3)
        
        # Calculate steering rates (degrees per second)
        steering_rates = []
        for i in range(1, len(filtered_angles)):
            dt = timestamps[i] - timestamps[i-1]
            if dt > 0:
                rate = abs(filtered_angles[i] - filtered_angles[i-1]) / dt
                steering_rates.append(rate)
        
        # Method 1: High steering rate detection (instantaneous)
        max_steering_rate = max(steering_rates) if steering_rates else 0
        avg_steering_rate = sum(steering_rates) / len(steering_rates) if steering_rates else 0
        
        # Method 2: Direction change analysis (improved)
        direction_changes = 0
        significant_changes = 0
        total_angle_change = 0.0
        rapid_reversals = 0
        
        # Use a minimum threshold to ignore noise
        NOISE_THRESHOLD = 1.0  # degrees - ignore changes smaller than this
        SIGNIFICANT_CHANGE_THRESHOLD = 5.0 
        
        for i in range(2, len(filtered_angles)):
            prev_angle = filtered_angles[i-2]
            curr_angle = filtered_angles[i-1]
            next_angle = filtered_angles[i]
            
            # Calculate direction vectors (with noise filtering)
            dir1 = curr_angle - prev_angle
            dir2 = next_angle - curr_angle
            
            # Only count if changes are above noise threshold
            if abs(dir1) > NOISE_THRESHOLD and abs(dir2) > NOISE_THRESHOLD:
                # Check for direction change (oscillation)
                if dir1 * dir2 < 0:  # Signs are different = direction change
                    direction_changes += 1
                    angle_change_magnitude = abs(dir1) + abs(dir2)
                    total_angle_change += angle_change_magnitude
                    
                    # Count significant direction changes
                    if abs(dir1) > SIGNIFICANT_CHANGE_THRESHOLD and abs(dir2) > SIGNIFICANT_CHANGE_THRESHOLD:
                        significant_changes += 1
                    
                    # Detect rapid reversals (large angle changes in short time)
                    time_span = timestamps[i] - timestamps[i-2]
                    if angle_change_magnitude > 15 and time_span < 1.0:  # >15° change in <1s
                        rapid_reversals += 1
        
        # Method 3: Standard deviation of steering angle (indicates erratic behavior)
        angle_std_dev = 0
        if len(filtered_angles) > 1:
            mean_angle = sum(filtered_angles) / len(filtered_angles)
            variance = sum((angle - mean_angle) ** 2 for angle in filtered_angles) / len(filtered_angles)
            angle_std_dev = variance ** 0.5
        
        # Time window for frequency calculation
        time_window = recent_steering_history[-1][0] - recent_steering_history[0][0]
        
        if time_window > 0:
            oscillation_frequency = direction_changes / time_window
            significant_frequency = significant_changes / time_window
            rapid_reversal_frequency = rapid_reversals / time_window
            avg_change_magnitude = total_angle_change / max(1, direction_changes)
            
            # Enhanced debug logging
            self._log_message(
                f"DEBUG Steering Analysis: "
                f"Max_rate={max_steering_rate:.1f}°/s, Avg_rate={avg_steering_rate:.1f}°/s, "
                f"Total_changes={direction_changes}, Significant={significant_changes}, "
                f"Rapid_reversals={rapid_reversals}, "
                f"Osc_freq={oscillation_frequency:.2f}/s, Sig_freq={significant_frequency:.2f}/s, "
                f"Avg_magnitude={avg_change_magnitude:.1f}°, Std_dev={angle_std_dev:.1f}°, "
                f"Window={time_window:.1f}s",
                current_timestamp
            )
            
            # Enhanced detection criteria
            jerky_detected = False
            penalty_reason = ""
            detection_method = ""
            
            # Method 1: Extremely high instantaneous steering rate
            if max_steering_rate > 60:  # >60 degrees per second
                penalty += min(30, max_steering_rate * 0.3)
                jerky_detected = True
                penalty_reason = f"Extreme steering rate: {max_steering_rate:.1f}°/s"
                detection_method = "High_Rate"
                
            # Method 2: High frequency of rapid reversals
            elif rapid_reversal_frequency > 0.5:  # More than 0.5 rapid reversals per second
                penalty += min(25, rapid_reversal_frequency * 40)
                jerky_detected = True
                penalty_reason = f"Rapid steering reversals: {rapid_reversal_frequency:.2f}/s"
                detection_method = "Rapid_Reversals"
                
            # Method 3: High standard deviation indicates erratic steering
            elif angle_std_dev > 20 and avg_steering_rate > 15:
                penalty += min(20, angle_std_dev * 0.8)
                jerky_detected = True
                penalty_reason = f"Erratic steering pattern: std={angle_std_dev:.1f}°, avg_rate={avg_steering_rate:.1f}°/s"
                detection_method = "Erratic_Pattern"
                
            # Method 4: High frequency of significant direction changes
            elif significant_frequency > 1.0 and avg_change_magnitude > 10:
                penalty += min(20, significant_frequency * 15)
                jerky_detected = True
                penalty_reason = f"Frequent significant oscillations: {significant_frequency:.2f}/s, avg {avg_change_magnitude:.1f}°"
                detection_method = "Frequent_Oscillations"
                
            # Method 5: Moderate but sustained oscillation
            elif oscillation_frequency > 2.0 and avg_change_magnitude > 5:
                penalty += min(15, oscillation_frequency * 5)
                jerky_detected = True
                penalty_reason = f"Sustained oscillation: {oscillation_frequency:.2f}/s"
                detection_method = "Sustained_Oscillation"
            
            # Apply penalty if detected and cooldown period has passed
            if jerky_detected and (current_timestamp - self.last_jerky_steering_event_time >= self.config['EVENT_COOLDOWN_SEC']):
                self._log_message(
                    f"SAFETY: Jerky Steering detected ({detection_method})! {penalty_reason}, Penalty={penalty:.2f}",
                    current_timestamp
                )
                self._send_event(self.safety_score, self.eco_score, f"Jerky steering detected: {penalty_reason}")
                self.last_jerky_steering_event_time = current_timestamp
            elif jerky_detected:
                # Reset penalty if still in cooldown
                penalty = 0.0
                self._log_message(f"DEBUG: Jerky steering detected ({detection_method}) but in cooldown: {penalty_reason}", current_timestamp)
            else:
                # Log when no jerky steering is detected for debugging
                if direction_changes > 0:
                    self._log_message(f"DEBUG: Normal steering detected - no jerky behavior", current_timestamp)

        return penalty

    def _detect_system_intervention_event(self, current_vsa_tcs_act_data, current_abs_ebd_act_data, current_timestamp):
        penalty = 0.0
        
        # CORRECTED: Use .value for these complex signals
        if current_vsa_tcs_act_data and current_vsa_tcs_act_data[1] == True and \
           (current_timestamp - self.last_vsa_abs_act_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            new_penalty = self.config['C_Intervention']
            penalty += new_penalty
            self._log_message(f"SAFETY: VSA/TCS Activation detected! Penalty={new_penalty:.2f}", current_timestamp)
            self._send_event(self.safety_score, self.eco_score, "JVSA/TCS Activation detected")

            self.last_vsa_abs_act_event_time = current_timestamp

        # CORRECTED: Use .value for these complex signals
        if current_abs_ebd_act_data and current_abs_ebd_act_data[1] == True and \
           (current_timestamp - self.last_vsa_abs_act_event_time >= self.config['EVENT_COOLDOWN_SEC']):
            new_penalty = self.config['C_Intervention']
            penalty += new_penalty
            self._log_message(f"SAFETY: ABS/EBD Activation detected! Penalty={new_penalty:.2f}", current_timestamp)
            self._send_event(self.safety_score, self.eco_score, "JVSA/TCS Activation detected")
            self.last_vsa_abs_act_event_time = current_timestamp

        return penalty