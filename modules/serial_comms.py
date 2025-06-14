import serial
import time
import json
import threading
from .shared_state import state

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

def open_pico():
    try:
        state.pico = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        time.sleep(2)  # Attendere inizializzazione
        state.pico.reset_input_buffer()
    except Exception as e:
        print(f"Pico connection error: {e}")
        state.pico = None

def try_write(command: dict):
    if not state.pico or not state.pico.is_open:
        return
    
    try:
        state.pico.write((json.dumps(command) + "\n").encode())
        state.pico.flush()
    except Exception as e:
        print(f"Write error: {e}")
        state.pico.close()
        state.pico = None

def handle_serial_message(message: str):
    # Gestione angoli
    if message.startswith("ANG|"):
        try:
            angles = json.loads(message[4:])
            with state.lock:
                state.angle_j1 = angles.get('j1', 0)
                state.angle_j2 = angles.get('j2', 0)
                state.angle_j3 = angles.get('j3', 0)
        except json.JSONDecodeError:
            print(f"Invalid ANG message: {message}")
    
    # Gestione posizione
    elif message.startswith("POS|"):
        try:
            position = json.loads(message[4:])
            with state.lock:
                state.x = position.get('BASE', 0)
                state.y = position.get('M1', 0)
                state.z = position.get('M2', 0)
        except json.JSONDecodeError:
            print(f"Invalid POS message: {message}")

def serial_reader():
    while True:
        if state.pico and state.pico.is_open:
            try:
                while state.pico.in_waiting > 0:
                    raw = state.pico.readline().decode().strip()
                    
                    # Gestisci messaggi speciali
                    if raw.startswith(("ANG|", "POS|")):
                        handle_serial_message(raw)
                    # Gestisci altri messaggi se necessario
                    elif raw:
                        print(f"Received: {raw}")
            except Exception as e:
                print(f"Serial read error: {e}")
                try:
                    state.pico.close()
                except:
                    pass
                state.pico = None
        time.sleep(0.01)

# Avvia il thread reader all'import
threading.Thread(target=serial_reader, daemon=True).start()