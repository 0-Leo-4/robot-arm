# modules/encoders.py
import time
import smbus2
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

    def set_channel(self, channel):
        """Seleziona il canale sul multiplexer"""
        try:
            self.bus.write_byte(TCA9548A_ADDR, channel)
            time.sleep(0.005)  # Aumenta il tempo di attesa
        except Exception as e:
            print(f"Channel error: {e}")
            return False
        return True

    def init_encoders(self):
        try:
            print("Inizializzazione encoder...")
            
            # Verifica connessione multiplexer
            self.set_channel(0)
            time.sleep(0.1)
            
            # Scansione dispositivi sui canali
            for i, channel in enumerate(CHANNELS):
                if not self.set_channel(channel):
                    continue
                
                try:
                    devices = self.bus.scan()
                    print(f"Canale {i} ({hex(channel)}): Dispositivi trovati {[hex(d) for d in devices]}")
                    
                    if AS5600_ADDR in devices:
                        print(f"  AS5600 rilevato all'indirizzo {hex(AS5600_ADDR)}")
                    else:
                        print(f"  AS5600 non rilevato sul canale {i}")
                except Exception as e:
                    print(f"Scan error: {e}")
            
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
                
            raw = self.bus.read_word_data(AS5600_ADDR, AS5600_RAW_ANGLE_REG)
            # I dati sono in big-endian? Il AS5600 restituisce 12 bit
            raw = ((raw << 8) & 0xFF00) | (raw >> 8)
            return raw & 0x0FFF
        except Exception as e:
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
            self.init_encoders()
            time.sleep(1)
        
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
                    
            except Exception as e:
                print(f"Encoder update error: {e}")
            
            time.sleep(0.05)  # 20 Hz

# Istanza globale
encoder_reader = EncoderReader()