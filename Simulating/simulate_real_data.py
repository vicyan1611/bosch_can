import math
import random
from dataclasses import dataclass
from typing import List, Tuple
import time
from scoring.DrivingScoreEvaluator import DrivingScoreEvaluator

@dataclass
class CANDataPacket:
    """Simulated CAN data packet matching your evaluator's expected structure"""
    timestamp: float
    
    # Eco-friendly signals (primitive types)
    ENG_DRIVER_REQ_TRQ_13C: float  # Nm, range: -6553.6 to 6553.4
    ENG_SMART_ACCELE_PEDAL_POS_13C: float  # %, range: 0 to 100
    VSA_ABS_FL_WHEEL_SPEED: float  # km/h, range: 0 to 327.67
    ENG_ENG_SPEED: float  # rpm, range: 0 to 65535
    CVT_GEAR_POSITION_IND_CVT: int  # range: 0 to 15
    ENG_IS_PROGRESS: bool  # Idle-stop system
    
    # Safety signals (based on your code, these seem to be complex types with .value)
    VSA_LON_G: float  # Longitudinal G-force, range: -24.5 to 24.452148949
    VSA_LAT_G: float  # Lateral G-force, range: -24.5 to 24.452148949  
    VSA_YAW_1: float  # Yaw rate, range: -125 to 124.75585938
    STR_ANGLE: float  # Steering angle in degrees, range: -3276.8 to 3276.7
    VSA_VSA_TCS_ACT: bool  # VSA/TCS activation flag
    VSA_ABS_EBD_ACT: bool  # ABS/EBD activation flag

class DrivingScenarioSimulator:
    """Generates realistic CAN data for different driving scenarios"""
    
    def __init__(self, sample_rate_hz: float = 50.0):
        self.sample_rate_hz = sample_rate_hz
        self.dt = 1.0 / sample_rate_hz
        self.current_time = 0.0
        
        # Vehicle state
        self.speed = 0.0  # km/h
        self.acceleration = 0.0  # m/s^2
        self.rpm = 800.0  # idle RPM
        self.gear = 1
        self.steering_angle = 0.0
        self.pedal_position = 0.0
        
    def generate_normal_driving(self, duration_sec: float) -> List[CANDataPacket]:
        """Generate normal, smooth driving data"""
        packets = []
        end_time = self.current_time + duration_sec
        
        while self.current_time < end_time:
            # Smooth speed changes
            target_speed = 50 + 10 * math.sin(self.current_time * 0.1)
            speed_diff = target_speed - self.speed
            self.acceleration = max(-2, min(2, speed_diff * 0.1))  # Gentle acceleration
            self.speed = max(0, self.speed + self.acceleration * 3.6 * self.dt)
            
            # RPM follows speed smoothly
            self.rpm = max(800, 800 + self.speed * 30)
            
            # Gentle steering
            self.steering_angle = 15 * math.sin(self.current_time * 0.05)
            
            # Pedal position based on acceleration
            self.pedal_position = max(0, min(100, 20 + self.acceleration * 10))
            
            packet = self._create_packet()
            packets.append(packet)
            
            self.current_time += self.dt
            
        return packets
    
    def generate_aggressive_acceleration(self, duration_sec: float) -> List[CANDataPacket]:
        """Generate hard acceleration events that should trigger safety penalties"""
        packets = []
        end_time = self.current_time + duration_sec
        
        while self.current_time < end_time:
            # Aggressive acceleration (should exceed 3.9 m/s^2 threshold)
            self.acceleration = 6.0 + random.uniform(-1, 1)  # ~0.6G acceleration
            self.speed = max(0, self.speed + self.acceleration * 3.6 * self.dt)
            
            # High RPM due to aggressive driving
            self.rpm = max(800, 2000 + self.speed * 40)
            
            # High pedal position
            self.pedal_position = 80 + random.uniform(-10, 15)
            
            # Some steering input
            self.steering_angle = 5 * math.sin(self.current_time * 0.3)
            
            packet = self._create_packet()
            packets.append(packet)
            
            self.current_time += self.dt
            
        return packets
    
    def generate_hard_braking(self, duration_sec: float) -> List[CANDataPacket]:
        """Generate hard braking events that should trigger safety penalties"""
        packets = []
        end_time = self.current_time + duration_sec
        
        while self.current_time < end_time:
            # Hard braking (should exceed -5.8 m/s^2 threshold)
            self.acceleration = -7.0 + random.uniform(-1, 1)  # ~0.7G braking
            self.speed = max(0, self.speed + self.acceleration * 3.6 * self.dt)
            
            # RPM drops during braking
            self.rpm = max(800, 800 + self.speed * 25)
            
            # Zero pedal during braking
            self.pedal_position = 0
            
            # Slight steering corrections
            self.steering_angle = 8 * math.sin(self.current_time * 0.4)
            
            packet = self._create_packet()
            packets.append(packet)
            
            self.current_time += self.dt
            
        return packets
    
    def generate_aggressive_cornering(self, duration_sec: float) -> List[CANDataPacket]:
        """Generate aggressive cornering with high lateral G and yaw rates"""
        packets = []
        end_time = self.current_time + duration_sec
        
        # Set a moderate speed for cornering
        self.speed = 60.0
        
        while self.current_time < end_time:
            # Maintain speed through corner
            self.acceleration = random.uniform(-0.5, 0.5)
            self.speed = max(0, self.speed + self.acceleration * 3.6 * self.dt)
            
            # RPM for constant speed
            self.rpm = 1500 + self.speed * 25
            
            # Aggressive steering input (high rate of change)
            steering_freq = 0.5  # Fast steering changes
            self.steering_angle = 45 * math.sin(self.current_time * steering_freq)
            
            # Moderate pedal to maintain speed
            self.pedal_position = 30 + random.uniform(-5, 5)
            
            packet = self._create_packet()
            packets.append(packet)
            
            self.current_time += self.dt
            
        return packets
    
    def generate_system_intervention(self, duration_sec: float) -> List[CANDataPacket]:
        """Generate scenarios where VSA/TCS or ABS systems activate"""
        packets = []
        end_time = self.current_time + duration_sec
        
        activation_time = self.current_time + duration_sec * 0.5  # Activate mid-way
        
        while self.current_time < end_time:
            # Unstable conditions leading to system intervention
            self.acceleration = random.uniform(-3, 3)
            self.speed = max(0, self.speed + self.acceleration * 3.6 * self.dt)
            
            self.rpm = max(800, 1000 + self.speed * 35)
            self.pedal_position = 50 + random.uniform(-20, 30)
            self.steering_angle = 20 * math.sin(self.current_time * 0.8)
            
            packet = self._create_packet()
            
            # Trigger system intervention
            if abs(self.current_time - activation_time) < 0.5:  # 1 second window
                packet.VSA_VSA_TCS_ACT = True
                packet.VSA_ABS_EBD_ACT = True
            
            packets.append(packet)
            self.current_time += self.dt
            
        return packets
    
    def generate_inefficient_driving(self, duration_sec: float) -> List[CANDataPacket]:
        """Generate inefficient driving patterns (high RPM, excessive idling, etc.)"""
        packets = []
        end_time = self.current_time + duration_sec
        
        while self.current_time < end_time:
            # Low speed but high RPM (inefficient)
            self.speed = 20 + 10 * math.sin(self.current_time * 0.1)
            self.acceleration = random.uniform(-1, 1)
            
            # Inefficiently high RPM for the speed
            self.rpm = 3000 + random.uniform(-200, 500)  # Way too high for low speed
            
            # High pedal position contributing to inefficiency
            self.pedal_position = 60 + random.uniform(-10, 20)
            
            # Frequent gear changes
            if int(self.current_time * 2) % 3 == 0:  # Change gear every 1.5 seconds
                self.gear = (self.gear % 6) + 1
            
            # Add some idling periods
            if int(self.current_time) % 10 < 3:  # 30% of time idling
                self.speed = 0
                self.rpm = 900  # High idle RPM
                self.pedal_position = 0
            
            self.steering_angle = 10 * math.sin(self.current_time * 0.2)
            
            packet = self._create_packet()
            packets.append(packet)
            
            self.current_time += self.dt
            
        return packets
    
    def _create_packet(self) -> CANDataPacket:
        """Create a CAN data packet with current vehicle state"""
        
        # Calculate lateral G and yaw based on steering and speed
        # Simplified physics model
        lat_g = 0.0
        yaw_rate = 0.0
        
        if self.speed > 5:  # Only calculate when moving
            # Lateral G from cornering (simplified)
            corner_radius = max(10, abs(self.steering_angle) + 1)  # Avoid division by zero
            lat_g = (self.speed / 3.6) ** 2 / corner_radius * 0.1  # Convert to m/s^2
            if self.steering_angle < 0:
                lat_g = -lat_g
                
            # Yaw rate from steering
            yaw_rate = self.steering_angle * 0.5  # Simplified relationship
        
        # Add some noise to make it realistic
        lat_g += random.uniform(-0.2, 0.2)
        yaw_rate += random.uniform(-2, 2)
        
        # Calculate torque request based on pedal position and load
        torque_request = self.pedal_position * 2.0 + random.uniform(-10, 10)
        
        return CANDataPacket(
            timestamp=self.current_time,
            
            # Eco signals
            ENG_DRIVER_REQ_TRQ_13C=torque_request,
            ENG_SMART_ACCELE_PEDAL_POS_13C=self.pedal_position,
            VSA_ABS_FL_WHEEL_SPEED=self.speed,
            ENG_ENG_SPEED=self.rpm,
            CVT_GEAR_POSITION_IND_CVT=self.gear,
            ENG_IS_PROGRESS=self.speed < 0.1 and random.random() < 0.1,  # Occasional idle-stop
            
            # Safety signals
            VSA_LON_G=self.acceleration,  # Longitudinal G
            VSA_LAT_G=lat_g,  # Lateral G
            VSA_YAW_1=yaw_rate,  # Yaw rate
            STR_ANGLE=self.steering_angle,  # Steering angle
            VSA_VSA_TCS_ACT=False,  # Will be set by specific scenarios
            VSA_ABS_EBD_ACT=False   # Will be set by specific scenarios
        )

def create_test_scenarios() -> List[Tuple[str, List[CANDataPacket]]]:
    """Create a comprehensive set of test scenarios"""
    
    simulator = DrivingScenarioSimulator(sample_rate_hz=20.0)  # 20 Hz sampling
    scenarios = []
    
    # Test Scenario 1: Normal driving (should maintain high scores)
    normal_data = simulator.generate_normal_driving(30.0)  # 30 seconds
    scenarios.append(("Normal Driving", normal_data))
    
    # Test Scenario 2: Aggressive acceleration (should reduce safety score)
    aggressive_accel_data = simulator.generate_aggressive_acceleration(10.0)
    scenarios.append(("Aggressive Acceleration", aggressive_accel_data))
    
    # Test Scenario 3: Hard braking (should reduce safety score)
    hard_brake_data = simulator.generate_hard_braking(8.0)
    scenarios.append(("Hard Braking", hard_brake_data))
    
    # Test Scenario 4: Aggressive cornering (should reduce safety score)
    aggressive_corner_data = simulator.generate_aggressive_cornering(15.0)
    scenarios.append(("Aggressive Cornering", aggressive_corner_data))
    
    # Test Scenario 5: System intervention (should significantly reduce safety score)
    system_intervention_data = simulator.generate_system_intervention(12.0)
    scenarios.append(("System Intervention", system_intervention_data))
    
    # Test Scenario 6: Inefficient driving (should reduce eco score)
    inefficient_data = simulator.generate_inefficient_driving(25.0)
    scenarios.append(("Inefficient Driving", inefficient_data))
    
    return scenarios

def run_test_scenario(evaluator, scenario_name: str, packets: List[CANDataPacket]):
    """Run a test scenario through the evaluator and print results"""
    
    print(f"\n{'='*60}")
    print(f"TESTING SCENARIO: {scenario_name}")
    print(f"{'='*60}")
    print(f"Duration: {len(packets) * 0.05:.1f} seconds")
    print(f"Data points: {len(packets)}")
    
    initial_safety = evaluator.safety_score
    initial_eco = evaluator.eco_score
    
    # Process all packets
    for packet in packets:
        eco_score, safety_score = evaluator.process_can_data(packet)
    
    final_safety = evaluator.safety_score
    final_eco = evaluator.eco_score
    
    print(f"\nSCORE CHANGES:")
    print(f"Safety Score: {initial_safety:.2f} → {final_safety:.2f} (Δ: {final_safety - initial_safety:.2f})")
    print(f"Eco Score: {initial_eco:.2f} → {final_eco:.2f} (Δ: {final_eco - initial_eco:.2f})")
    
    # Sample some data points for inspection
    print(f"\nSAMPLE DATA POINTS:")
    sample_indices = [0, len(packets)//4, len(packets)//2, 3*len(packets)//4, -1]
    
    for i in sample_indices:
        packet = packets[i]
        print(f"t={packet.timestamp:.1f}s: Speed={packet.VSA_ABS_FL_WHEEL_SPEED:.1f}km/h, "
              f"LonG={packet.VSA_LON_G:.2f}, LatG={packet.VSA_LAT_G:.2f}, "
              f"Yaw={packet.VSA_YAW_1:.1f}, RPM={packet.ENG_ENG_SPEED:.0f}")

# Example usage function
def main():
    """Main function demonstrating how to use the simulator"""
    
    # You'll need to import your DrivingScoreEvaluator here
    # from your_module import DrivingScoreEvaluator
    
    print("CAN Data Simulator for Driving Score Testing")
    print("="*50)
    
    # Create test scenarios
    scenarios = create_test_scenarios()
    
    print(f"Generated {len(scenarios)} test scenarios:")
    for i, (name, packets) in enumerate(scenarios, 1):
        print(f"{i}. {name}: {len(packets)} data points ({len(packets) * 0.05:.1f}s)")
    
    # Example of how to use with your evaluator:
    evaluator = DrivingScoreEvaluator()
    
    for scenario_name, packets in scenarios:
        run_test_scenario(evaluator, scenario_name, packets)
        
        # Reset evaluator state between scenarios if needed
        evaluator.safety_score = 100
        evaluator.eco_score = 100
        evaluator.current_safety_window_penalty_sum = 0.0
        # Reset other state variables as needed
    
    evaluator.close_log()

if __name__ == "__main__":
    main()