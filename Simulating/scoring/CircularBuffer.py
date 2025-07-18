from collections import deque

class CircularBuffer:
    def __init__(self, capacity_seconds):
        self.buffer = deque()
        self.capacity_seconds = capacity_seconds

    def add(self, item):
        # item is expected to be a (timestamp, value) tuple
        self.buffer.append(item)

    def trim_older_than(self, oldest_allowed_timestamp):
        while self.buffer and self.buffer[0][0] < oldest_allowed_timestamp:
            self.buffer.popleft()

    def get_all_items(self):
        # Returns all (timestamp, value) pairs currently in the buffer
        return list(self.buffer)

    def get_values_only(self):
        # Returns just the values from the (timestamp, value) pairs
        return [item[1] for item in self.buffer]
    
    def get_last_value(self):
        # Returns the last (timestamp, value) item, or None if buffer is empty
        return self.buffer[-1] if self.buffer else None

    def __len__(self):
        return len(self.buffer)

    def __getitem__(self, index):
        return self.buffer[index]