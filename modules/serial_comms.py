import serial
import time
import json
import threading
from .shared_state import state

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
CONNECTION_RETRY_DELAY = 2

def open_pico():
    max_attempts = 5
    for attempt in range(max_attempts):
        try:
            print(f"Connecting to Pico (attempt {attempt+1}/{max_attempts})")
            # Reduce timeout to prevent hangs
            state.pico = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
            time.sleep(1)  # Reduced initialization time
            state.pico.reset_input_buffer()
            print("Pico connection established")
            return True
        except Exception as e:
            print(f"Pico connection error: {e}")
            state.pico = None
            if attempt < max_attempts - 1:
                time.sleep(CONNECTION_RETRY_DELAY)
    
    print(f"Failed to connect to Pico after {max_attempts} attempts")
    return False

def try_write(command: dict):
    """Invia un comando JSON al Pico."""
    if not state.pico or not state.pico.is_open:
        return False
        
    try:
        cmd_str = json.dumps(command) + "\n"
        state.pico.write(cmd_str.encode())
        state.pico.flush()
        return True
    except Exception as e:
        print(f"Write error: {e}")
        try:
            state.pico.close()
        except:
            pass
        state.pico = None
        return False

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
    last_reconnect = 0
    while True:
        # Handle disconnections with backoff
        if not state.pico or not state.pico.is_open:
            current_time = time.time()
            if current_time - last_reconnect > CONNECTION_RETRY_DELAY:
                if open_pico():
                    last_reconnect = current_time
                else:
                    time.sleep(CONNECTION_RETRY_DELAY)
            continue
        
        try:
            # Read with timeout
            raw = state.pico.readline().decode().strip()
            if not raw:
                time.sleep(0.01)
                continue
                
            if raw.startswith(("ANG|", "POS|")):
                handle_serial_message(raw)
            else:
                print(f"Received: {raw}")
                
        except Exception as e:
            print(f"Serial read error: {e}")
            try:
                state.pico.close()
            except:
                pass
            state.pico = None

# Avvia il thread reader all'import
threading.Thread(target=serial_reader, daemon=True).start()