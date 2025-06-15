# modules/motion_control.py
import re
import time
import threading
from . import serial_comms, kinematics
from .shared_state import state

AXIS_MAP = {'X': 'BASE', 'Y': 'M1', 'Z': 'M2'}
STEP_PER_MM = 160

def parse_gcode_to_moves(code_block, mode='absolute'):
    pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
    moves = []

    for line in code_block.splitlines():
        line = line.split(';',1)[0].strip()
        m = re.match(r'^(G0|G1)\s*(.*)', line, re.IGNORECASE)
        if not m: continue
        
        coords = {}
        for letter, num in re.findall(r'([XYZF])([-+]?[0-9]*\.?[0-9]+)', m.group(2), re.IGNORECASE):
            if letter.upper() in ('X','Y','Z'):
                coords[letter.upper()] = float(num)

        deltas = {}
        for axis in ('X','Y','Z'):
            if axis in coords:
                tgt = coords[axis]
                if mode == 'absolute':
                    delta = tgt - pos[axis]
                    pos[axis] = tgt
                else:
                    delta = coords[axis]
                    pos[axis] += delta
                deltas[axis] = delta

        if not deltas:
            continue

        steps = {axis: int(abs(deltas[axis]) * STEP_PER_MM) for axis in deltas}
        max_steps = max(steps.values())
        err = {axis: 0 for axis in steps}

        for _ in range(max_steps):
            for axis, n_steps in steps.items():
                err[axis] += n_steps
                if err[axis] >= max_steps:
                    sign = 1 if deltas[axis] >= 0 else -1
                    moves.append({
                        "axis": AXIS_MAP[axis],
                        "mm": sign * (1.0 / STEP_PER_MM),
                        "speed_pct": state.current_speed
                    })
                    err[axis] -= max_steps
    return moves

def motion_handler():
    """Gestisce la coda di comandi di movimento"""
    while True:
        if state.command_queue and not state.emergency_active:
            cmd = state.command_queue.pop(0)
            serial_comms.try_write(cmd)
            time.sleep(0.05)
        else:
            time.sleep(0.1)
               
def execute_sequence():
    """Executes the pick-and-place sequence"""
    state.sequence_running = True
    state.sequence_interrupt = False
    
    try:
        # 1. Open gripper (safety)
        state.command_queue.append({"cmd": "grip", "angle": 0})
        
        # 2. Get detection data
        with state.lock:
            detections = state.detections.copy()
        
        if not detections:
            return
        
        # 3. Find circle with ID 1
        target_circle = next((d for d in detections if d.get('id') == 1), None)
        if not target_circle:
            return
        
        # 4. Convert to robot coordinates (simple scaling - should be calibrated)
        x_img, y_img = target_circle['x'], target_circle['y']
        x_robot = x_img * 0.5  # Placeholder conversion
        y_robot = y_img * 0.5  # Placeholder conversion
        z_robot = 0
        
        # Apply gripper offset
        x_robot += state.gripper_offset[0]
        y_robot += state.gripper_offset[1]
        z_robot += state.gripper_offset[2]
        
        # 5. Move to target position
        target_angles = kinematics.inverse_kinematics(
            x_robot, y_robot, z_robot
        )
        move_to_angles(target_angles)
        
        if state.sequence_interrupt:
            return
        
        # 6. Close gripper
        state.command_queue.append({"cmd": "grip", "angle": 84})
        time.sleep(1)
        
        # 7. Move to drop position
        drop_angles = kinematics.inverse_kinematics(
            state.drop_position[0],
            state.drop_position[1],
            state.drop_position[2]
        )
        move_to_angles(drop_angles)
        
        if state.sequence_interrupt:
            return
        
        # 8. Open gripper
        state.command_queue.append({"cmd": "grip", "angle": 0})
        time.sleep(1)
        
        # 9. Return to home
        move_to_angles(state.home_angles)
        
    finally:
        state.sequence_running = False

def move_to_angles(target_angles):
    """Move joints to target angles"""
    current_angles = [state.angle_j1, state.angle_j2, state.angle_j3]
    axes = ['BASE', 'M1', 'M2']
    
    for i, (current, target) in enumerate(zip(current_angles, target_angles)):
        diff = target - current
        if abs(diff) > 1.0:  # Only move if significant difference
            state.command_queue.append({
                'axis': axes[i],
                'mm': diff,
                'speed_pct': state.current_speed
            })

def sequence_handler():
    """Handles sequence execution"""
    while True:
        if state.start_sequence and state.homing_done and not state.sequence_running:
            threading.Thread(target=execute_sequence, daemon=True).start()
            state.start_sequence = False
        time.sleep(0.1)

# Start the sequence handler thread
threading.Thread(target=sequence_handler, daemon=True).start()