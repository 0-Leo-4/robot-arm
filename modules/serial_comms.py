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
    try:
        if message.startswith("ALL|"):
            data = json.loads(message[4:])
            with state.lock:
                # Aggiorna angoli con fallback
                angles = data.get('angles', {})
                state.angle_j1 = angles.get('j1', state.angle_j1)
                state.angle_j2 = angles.get('j2', state.angle_j2)
                state.angle_j3 = angles.get('j3', state.angle_j3)
                
                # Aggiorna posizione con fallback
                position = data.get('position', {})
                state.x = position.get('BASE', state.x)
                state.y = position.get('M1', state.y)
                state.z = position.get('M2', state.z)
                
                # Aggiorna timestamp ultimo aggiornamento
                state.last_update = time.time()
        elif message.startswith("ANG|"):
            try:
                angles = json.loads(message[4:])
                with state.lock:
                    state.angle_j1 = angles.get('j1', state.angle_j1)
                    state.angle_j2 = angles.get('j2', state.angle_j2)
                    state.angle_j3 = angles.get('j3', state.angle_j3)
                    state.last_angle_update = time.time()
            except json.JSONDecodeError:
                print(f"Invalid ANG message: {message[:50]}...")
        elif message.startswith("POS|"):
            try:
                position = json.loads(message[4:])
                with state.lock:
                    state.x = position.get('BASE', state.x)
                    state.y = position.get('M1', state.y)
                    state.z = position.get('M2', state.z)
            except json.JSONDecodeError:
                print(f"Invalid POS message: {message[:50]}...")
    except Exception as e:
        print(f"Error handling message: {e}")

def request_pico_data():
    """Richiede tutti i dati al Pico in un unico comando"""
    if state.pico and state.pico.is_open:
        try_write({"cmd": "getall"})

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
                if raw.startswith(("ALL|", "ANG|", "POS|")):
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

def periodic_data_request():
    """Richiede periodicamente tutti i dati al Pico"""
    while True:
        if state.pico and state.pico.is_open:
            request_pico_data()
        time.sleep(0.15)  # 150ms per ridurre il carico

# Avvia i thread all'import
threading.Thread(target=serial_reader, daemon=True).start()
threading.Thread(target=periodic_data_request, daemon=True).start()