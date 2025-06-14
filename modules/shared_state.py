import threading
from time import time
from gpiozero import OutputDevice


class SharedState:
    def __init__(self):
        self.detections = []
        self.current_speed = 100
        self.emergency_active = False
        self.latest_frame = None
        self.lock = threading.Lock()
        self.lcd_lock = threading.Lock()
        self.tracked_circles = {}
        self.next_circle_id = 1
        self.max_lost_distance = 50
        self.calibration_matrix = None
        self.calibration_points = []
        self.calibration_active = False
        self.calibration_object = None
        self.command_queue = []
        self.relay = OutputDevice(17, active_high=False, initial_value=True)
        self.pico = None
        self.conveyor_speed = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle_j1 = 0
        self.angle_j2 = 0
        self.angle_j3 = 0
        self.start_sequence = False
        self.last_angle_update = time.time()
        self.serial_lock = threading.Lock()  # lock for serial
        # connection tracking
        self.last_serial_error = 0
        self.serial_error_count = 0

    @property
    def angles_fresh(self):
        return time.time() - self.last_angle_update < 1.0

state = SharedState()