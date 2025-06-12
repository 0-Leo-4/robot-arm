# modules/serial_comms.py
import serial
import time
import json
from .shared_state import state

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

def open_pico():
    try:
        port = SERIAL_PORT
        state.pico = serial.Serial(port, BAUDRATE, timeout=0.1, write_timeout=0.1)
        time.sleep(2)
        state.pico.reset_input_buffer()
        state.pico.reset_output_buffer()
    except Exception as e:
        state.pico = None
        print(f"Impossibile aprire Pico: {e}")

def monitor_pico():
    while True:
        if state.pico:
            try: 
                _ = state.pico.in_waiting
            except Exception:
                state.pico.close()
                state.pico = None
        time.sleep(1)

def try_write(cmd: dict):
    with state.lock:
        if not state.pico or not state.pico.is_open: 
            return
        try:
            state.pico.reset_output_buffer()
            state.pico.write((json.dumps(cmd) + "\n").encode())
            state.pico.flush()
        except Exception:
            state.pico.close()
            state.pico = None