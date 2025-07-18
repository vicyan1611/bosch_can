

class CANDataPackage:
    """
    Lớp chứa một "ảnh chụp" (snapshot) của tất cả các tín hiệu CAN liên quan 
    tại một mốc thời gian cụ thể, với các giá trị mặc định và một phương thức cập nhật.
    """
    
    _DEFAULT_SIGNALS = {
        'ENG_DRIVER_REQ_TRQ_13C': 0.0,
        'ENG_SMART_ACCELE_PEDAL_POS_13C': 0.0,
        'VSA_ABS_FL_WHEEL_SPEED': 0.0,
        'ENG_ENG_SPEED': 0.0,
        'CVT_GEAR_POSITION_IND_CVT': 'P',
        'ENG_IS_PROGRESS': False,
        'VSA_LON_G': 0.0,
        'VSA_LAT_G': 0.0,
        'VSA_YAW_1': 0.0,
        'STR_ANGLE': 0.0,
        'VSA_VSA_TCS_ACT': False,
        'VSA_ABS_EBD_ACT': False,
    }

    def __init__(self, timestamp, **kwargs):
        """
        Khởi tạo gói dữ liệu.
        Tất cả các tín hiệu sẽ được thiết lập giá trị mặc định trước,
        sau đó được cập nhật bằng bất kỳ giá trị nào được cung cấp trong kwargs.
        
        :param timestamp: Dấu thời gian của gói dữ liệu.
        :param kwargs: Các giá trị tín hiệu ban đầu.
        """
        self.timestamp = timestamp
        
        # 1. Thiết lập tất cả các tín hiệu về giá trị mặc định
        for key, value in self._DEFAULT_SIGNALS.items():
            setattr(self, key, value)
            
        # 2. Cập nhật các giá trị được cung cấp khi khởi tạo
        self.update(timestamp, **kwargs)

    def update(self, timestamp, **kwargs):
        """
        Cập nhật gói dữ liệu với các giá trị tín hiệu mới.
        Chỉ các tín hiệu được cung cấp trong kwargs mới được cập nhật.
        Các tín hiệu khác sẽ giữ nguyên giá trị hiện tại của chúng.
        
        :param timestamp: Dấu thời gian của bản cập nhật.
        :param kwargs: Các giá trị tín hiệu mới cần cập nhật.
        """
        self.timestamp = timestamp
        for key, value in kwargs.items():
            # setattr sẽ ghi đè thuộc tính nếu nó đã tồn tại,
            # hoặc tạo mới nếu chưa có.
            setattr(self, key, value)

    def __repr__(self):
        """
        Hiển thị một cách thân thiện để gỡ lỗi (debug).
        """
        signal_data = ", ".join(f"{k}={v}" for k, v in self.to_dict().items() if k != 'timestamp')
        return f"CANDataPackage(ts={self.timestamp}, {signal_data})"

    def to_dict(self):
        """
        Chuyển đổi tất cả các thuộc tính thành một từ điển.
        """
        return {key: getattr(self, key) for key in self._DEFAULT_SIGNALS.keys() | {'timestamp'}}