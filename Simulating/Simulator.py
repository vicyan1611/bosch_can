from can import ASCReader
from CANDataAdapter import CANDataAdapter
from scoring.CANDataPackage import CANDataPackage
import cantools
import can
from scoring.DrivingScoreEvaluator import DrivingScoreEvaluator
import time 

DBC_FILE = 'data/BOSCH_CAN.dbc'
ASC_FILE = 'data/CANWIN.asc'
CAN_INTERFACE = 'vcan0'

class Simulator:
    def __init__(self):
        """
        Initialize the simulator with CAN data.
        :param can_data: A list of CANDataPackage objects.
        """
        self.adapter = CANDataAdapter()
        self.evaluator = DrivingScoreEvaluator()
        self.safety_scores_log = []
        self.eco_scores_log = []
        self.eco_timestamps_log = []
        self.safety_timestamps_log = []

    def run_simulation(self):
        try:
            db = cantools.db.load_file(DBC_FILE)
            bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
            bus.set_filters([
                {"can_id": 0x13C, "can_mask": 0x7FF},
                {"can_id": 0x1D0, "can_mask": 0x7FF},
                {"can_id": 0x191, "can_mask": 0x7FF},
                {"can_id": 0x17C, "can_mask": 0x7FF},
                {"can_id": 0x091, "can_mask": 0x7FF},
            ])
            print(f"Detector started. Listening on {CAN_INTERFACE}...")
        except FileNotFoundError:
            print(f"Error: DBC file '{DBC_FILE}' not found.")
            exit()

        timeout = 10.0  # seconds without any messages
        start_time = time.time()
        running = True

        while running:
            messages = self._recv_many(bus, timeout=timeout)

            if messages:
                start_time = time.time()  # Reset timer when messages are received

                for msg in messages:
                    # print(f"Received Message with ID: {hex(msg.arbitration_id)}")

                    try:
                        decoded_data = db.decode_message(msg.arbitration_id, msg.data, decode_choices=False)
                    except Exception as e:
                        print(f"Failed to decode message {hex(msg.arbitration_id)}: {e}")
                        continue

                    normalized_data = {
                        k: v.value if hasattr(v, "value") else v for k, v in decoded_data.items()
                    }

                    timestamp = msg.timestamp
                    self.adapter.msg_to_package(timestamp, normalized_data)

                    can_package = self.adapter.get_data_package()
                    eco_score, safety_score = self.evaluator.process_can_data(can_package)

                    if eco_score is not None:
                        self.eco_scores_log.append(eco_score)
                        self.eco_timestamps_log.append(timestamp)
                        print(f"Time: {timestamp:.1f}s | Eco Score: {eco_score:.2f}")

                    if safety_score is not None:
                        self.safety_scores_log.append(safety_score)
                        self.safety_timestamps_log.append(timestamp)
                        print(f"Time: {timestamp:.1f}s | Safety Score: {safety_score:.2f}")
            else:
                if time.time() - start_time > timeout:
                    print("No messages received for 10 seconds. Stopping.")
                    running = False


    def plot_results(self):
        """
        Plot the results of the simulation.
        """
        import matplotlib.pyplot as plt

        plt.subplot(2, 1, 1) # 2 rows, 1 column, first plot
        plt.plot(self.eco_timestamps_log, self.eco_scores_log, label='Eco Score', color='green')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Eco Score (0-100)')
        plt.title('Eco-Friendly Driving Score Evolution')
        plt.legend()
        plt.grid(True)

        plt.subplot(2, 1, 2) # 2 rows, 1 column, second plot
        plt.plot(self.safety_timestamps_log, self.safety_scores_log, label='Safety Score', color='red')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Safety Score (0-100)')
        plt.title('Safety Driving Score Evolution')
        plt.legend()
        plt.grid(True)

        plt.tight_layout() # Adjust layout to prevent overlapping
        plt.show()

    def _recv_many(bus, timeout=0.5, max_msgs=50):
        """
        Fallback function to collect multiple messages within a timeout.
        """
        from time import time
        messages = []
        start = time()
        while time() - start < timeout and len(messages) < max_msgs:
            msg = bus.recv(timeout=0.1)
            if msg:
                messages.append(msg)
        return messages
    
    def run_simulation_local(self):
        try:
            db = cantools.db.load_file(DBC_FILE)
            print("DBC loaded.")
        except FileNotFoundError:
            print(f"Error: DBC file '{DBC_FILE}' not found.")
            exit()

        # Path to your CANWIN.asc file
        print(f"Loading CAN log from {ASC_FILE}...")

        # Only keep necessary CAN IDs
        allowed_ids = {0x13C, 0x1D0, 0x191, 0x17C, 0x091}

        # Read messages from .asc file
        log = ASCReader(ASC_FILE)
        messages = list(log)

        print(f"{len(messages)} messages loaded from log.")

        # Simulate real-time replay
        last_time = None

        for msg in messages:
            if msg.arbitration_id not in allowed_ids:
                continue  # Skip irrelevant messages

            if last_time is not None:
                sleep_time = msg.timestamp - last_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
            last_time = msg.timestamp

            try:
                decoded_data = db.decode_message(msg.arbitration_id, msg.data, decode_choices=False)
            except Exception as e:
                print(f"Failed to decode message {hex(msg.arbitration_id)}: {e}")
                continue

            normalized_data = {
                k: v.value if hasattr(v, "value") else v for k, v in decoded_data.items()
            }

            timestamp = msg.timestamp
            self.adapter.msg_to_package(timestamp, normalized_data)

            can_package = self.adapter.get_data_package()
            eco_score, safety_score = self.evaluator.process_can_data(can_package)

            if eco_score is not None:
                self.eco_scores_log.append(eco_score)
                self.eco_timestamps_log.append(timestamp)
                print(f"Time: {timestamp:.1f}s | Eco Score: {eco_score:.2f}")

            if safety_score is not None:
                self.safety_scores_log.append(safety_score)
                self.safety_timestamps_log.append(timestamp)
                print(f"Time: {timestamp:.1f}s | Safety Score: {safety_score:.2f}")
