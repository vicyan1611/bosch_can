

class CANDataPackage:
    def __init__(self, timestamp, **kwargs):
        self.timestamp = timestamp
        # Dynamically set attributes for each signal
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __repr__(self):
        # A friendly representation for debugging
        signal_data = ", ".join(f"{k}={getattr(self, k)}" for k in self.__dict__ if k != 'timestamp')
        return f"CANSIGNALS(ts={self.timestamp}, {signal_data})"

    def to_dict(self):
        # Convert all attributes to a dictionary for CSV export
        return {k: getattr(self, k) for k in self.__dict__}