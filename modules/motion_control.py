# modules/motion_control.py
import re
import time
from . import serial_comms
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