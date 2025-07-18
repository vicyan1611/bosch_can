from scoring.CANDataPackage import CANDataPackage
from cantools.database.namedsignalvalue import NamedSignalValue

class CANDataAdapter:
    def __init__(self):
        self.data_package = CANDataPackage(0)

    def msg_to_package(self, timestamp, decoded_data):
        """
        Convert a CAN message to a CANDataPackage.
        :param decoded_data: A decoded object.
        :return: A CANDataPackage object.
        """
        normalized_data = {
            k: v.value if isinstance(v, NamedSignalValue) else v
            for k, v in decoded_data.items()
        }
        self.data_package.update(timestamp, **normalized_data)

    def get_data_package(self):
        return self.data_package