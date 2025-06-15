# modules/shared_state.py
import threading
from gpiozero import OutputDevice

class SharedState:
    def __init__(self):
        # Existing state variables
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
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.angle_j1 = 0.0
        self.angle_j2 = 0.0
        self.angle_j3 = 0.0
        self.start_sequence = False
        self.last_update = 0
        self.fps = 0.0
        
        # New state variables for sequence control
        self.homing_done = False
        self.sequence_running = False
        self.sequence_interrupt = False
        self.gripper_offset = (0, 0, 10)  # (x_offset, y_offset, z_offset) in mm
        self.drop_position = (100, 0, 0)   # Default drop position (x, y, z) in mm
        self.home_angles = [10.0, 5.0, 0.0]  # Home angles for BASE, M1, M2

state = SharedState()