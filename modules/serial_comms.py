import serial
import time
import json
import threading
from .shared_state import state

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

def open_pico():
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            print(f"Connecting to Pico (attempt {attempt+1}/{max_attempts})")
            state.pico = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
            time.sleep(2)  # Wait for initialization
            state.pico.reset_input_buffer()
            print("Pico connection established")
            return
        except Exception as e:
            print(f"Pico connection error: {e}")
            state.pico = None
            if attempt < max_attempts - 1:
                time.sleep(1)  # Wait before retrying
    
    print(f"Failed to connect to Pico after {max_attempts} attempts")

def try_write(command: dict):
    """Invia un comando JSON al Pico."""
    if not state.pico or not state.pico.is_open:
        return
    try:
        state.pico.write((json.dumps(command) + "\n").encode())
        state.pico.flush()
    except Exception as e:
        print(f"Write error: {e}")
        try:
            state.pico.close()
        except:
            pass
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
        # Logica di riconnessione automatica
        if not state.pico or not state.pico.is_open:
            open_pico()
            time.sleep(1)  # Aspetta prima di riprovare
            continue       # Torna all'inizio del loop
        
        try:
            while state.pico.in_waiting > 0:
                raw = state.pico.readline().decode().strip()
                if raw.startswith(("ANG|", "POS|")):
                    handle_serial_message(raw)
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