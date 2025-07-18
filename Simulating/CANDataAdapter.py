from scoring.CANDataPackage import CANDataPackage

class CANDataAdapter:
    def __init__(self, can_data):
        self.can_data = can_data

    def msg_to_package(self, decoded_data):
        """
        Convert a CAN message to a CANDataPackage.
        :param decoded_data: A decoded object.
        :return: A CANDataPackage object.
        """
        timestamp = decoded_data['timestamp']
        data = {k: v for k, v in decoded_data.items() if k != 'timestamp'}
        return CANDataPackage(timestamp, **data)