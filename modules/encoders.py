# modules/encoders.py
import time
import smbus2
import threading
from .shared_state import state
from . import kinematics

# Configurazione TCA9548A
TCA9548A_ADDR = 0x70
AS5600_ADDR = 0x36
CHANNELS = [0x01, 0x02, 0x04]  # Canali per BASE, M1, M2

# Registri AS5600
AS5600_RAW_ANGLE_REG = 0x0C

class EncoderReader:
    def __init__(self):
        self.bus = smbus2.SMBus(1)  # I2C bus 1
        self.last_angles = [0.0, 0.0, 0.0]  # BASE, M1, M2
        self.encoders_initialized = False
        self.lock = threading.Lock()
        self.error_count = 0
        self.max_errors = 10

    def set_channel(self, channel):
        """Seleziona il canale sul multiplexer"""
        try:
            with self.lock:
                self.bus.write_byte(TCA9548A_ADDR, channel)
            time.sleep(0.01)  # Aumenta il tempo di attesa
            return True
        except Exception as e:
            print(f"Channel error: {e}")
            return False

    def check_device_present(self, address):
        """Verifica se un dispositivo è presente all'indirizzo specificato"""
        try:
            with self.lock:
                self.bus.read_byte(address)
            return True
        except:
            return False

    def init_encoders(self):
        try:
            print("Inizializzazione encoder...")
            
            # Verifica connessione multiplexer
            if not self.set_channel(0):
                print("Errore selezione canale 0")
                return False
            
            # Verifica presenza dispositivi su ogni canale
            for i, channel in enumerate(CHANNELS):
                if not self.set_channel(channel):
                    print(f"Errore selezione canale {i} ({hex(channel)})")
                    continue
                
                present = self.check_device_present(AS5600_ADDR)
                print(f"Canale {i} ({hex(channel)}): AS5600 {'presente' if present else 'assente'}")
            
            self.encoders_initialized = True
            print("Encoders initialized successfully")
            return True
        except Exception as e:
            print(f"Encoder init error: {e}")
            return False

    def read_raw_angle(self, channel):
        try:
            if not self.set_channel(channel):
                return 0
                
            with self.lock:
                # Legge 2 byte dal registro dell'angolo grezzo
                raw_data = self.bus.read_i2c_block_data(AS5600_ADDR, AS5600_RAW_ANGLE_REG, 2)
            
            # Combina i 2 byte (big-endian)
            raw_angle = (raw_data[0] << 8) | raw_data[1]
            return raw_angle & 0x0FFF  # Mantiene solo 12 bit
        except Exception as e:
            self.error_count += 1
            if self.error_count <= self.max_errors:
                print(f"Error reading channel {hex(channel)}: {e}")
            return 0

    def read_angle(self, channel_idx):
        channel = CHANNELS[channel_idx]
        raw = self.read_raw_angle(channel)
        # Converti in gradi (0-360)
        angle = raw * 360.0 / 4096.0
        return angle

    def update_angles(self):
        angles = []
        for i in range(3):  # Per BASE, M1, M2
            angle = self.read_angle(i)
            # Filtraggio: se la variazione è troppo grande, scarta (rumore)
            if abs(angle - self.last_angles[i]) < 30:  # Soglia arbitraria
                angles.append(angle)
                self.last_angles[i] = angle
            else:
                angles.append(self.last_angles[i])
        return angles

    def run(self):
        while not self.encoders_initialized:
            if not self.init_encoders():
                print("Ritento inizializzazione encoder in 5 secondi...")
                time.sleep(5)
        
        while True:
            try:
                angles = self.update_angles()
                # Calcola la posizione cartesiana
                x, y, z = kinematics.forward_kinematics(angles)
                
                with state.lock:
                    state.angle_j1 = angles[0]
                    state.angle_j2 = angles[1]
                    state.angle_j3 = angles[2]
                    state.x = x
                    state.y = y
                    state.z = z
                    
                # Resetta il conteggio errori dopo una lettura valida
                self.error_count = 0
                
            except Exception as e:
                print(f"Encoder update error: {e}")
            
            time.sleep(0.05)  # 20 Hz

# Istanza globale
encoder_reader = EncoderReader()