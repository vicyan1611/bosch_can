from CANDataAdapter import CANDataAdapter
from scoring.CANDataPackage import CANDataPackage
import cantools
import can
from scoring.DrivingScoreEvaluator import DrivingScoreEvaluator
import time 

DBC_FILE = 'BOSCH_CAN.dbc'
CAN_INTERFACE = 'vcan0'

class Simulator:
    def __init__(self, can_data):
        """
        Initialize the simulator with CAN data.
        :param can_data: A list of CANDataPackage objects.
        """
        self.can_data = can_data
        self.adapter = CANDataAdapter(can_data)
        self.evaluator = DrivingScoreEvaluator()
        self.safety_scores_log = []
        self.eco_scores_log = []
        self.eco_timestamps_log = []
        self.safety_timestamps_log = []

    def run_simulation(self):
        """
        Run the simulation by processing each CAN data package.
        """
        
        try:
            db = cantools.db.load_file(DBC_FILE)
            bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')
            print(f"Detector started. Listening on {CAN_INTERFACE}...")
        except FileNotFoundError:
            print(f"Error: DBC file '{DBC_FILE}' not found.")
            exit()

        timeout = 5.0  # seconds
        start_time = time.time()
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                if time.time() - start_time > timeout:
                    print("No messages received for 5 seconds. Stopping.")
                    break
                continue
            start_time = time.time()
            decoded_data = db.decode_message(msg.arbitration_id, msg.data)
            can_package = self.adapter.msg_to_package(decoded_data)

            eco_score, safety_score = self.evaluator.process_can_data(can_package)
            current_sim_time = can_package['timestamp']

            if eco_score is not None:
                self.eco_scores_log.append(eco_score)
                self.eco_timestamps_log.append(current_sim_time)
                print(f"Time: {current_sim_time:.1f}s | Eco Score: {eco_score:.2f}")

            if safety_score is not None:
                self.safety_scores_log.append(safety_score)
                self.safety_timestamps_log.append(current_sim_time)
                print(f"Time: {current_sim_time:.1f}s | Safety Score: {safety_score:.2f}")


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
