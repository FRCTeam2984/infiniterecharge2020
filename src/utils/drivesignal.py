class DriveSignal:
    def __init__(
        self, left: float = 0.0, right: float = 0.0, brake_mode: float = False
    ):
        self.left = left
        self.right = right
        self.brake_mode = brake_mode
