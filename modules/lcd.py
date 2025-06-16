# modules/lcd.py
import time
import smbus2
from .shared_state import state

LCD_I2C_ADDR = 0x27
TCA9548A_ADDR = 0x70
LCD_CHANNEL = 0x08  # Canale 4 del multiplexer
I2C_BUS = 1

class LCDController:
    def __init__(self):
        self.bus = smbus2.SMBus(I2C_BUS)
        self.backlight = True
        
    def _set_channel(self):
        """Seleziona il canale LCD sul multiplexer"""
        try:
            self.bus.write_byte(TCA9548A_ADDR, LCD_CHANNEL)
            time.sleep(0.001)
        except Exception as e:
            print(f"LCD channel error: {e}")

    def write_command(self, cmd):
        """Invia un comando al display"""
        self._set_channel()
        try:
            # Comando su entrambi i nibble (4-bit mode)
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x00, (cmd & 0xF0) | 0x04)
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x00, (cmd & 0xF0))
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x00, ((cmd << 4) & 0xF0) | 0x04)
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x00, ((cmd << 4) & 0xF0))
            time.sleep(0.001)
        except Exception as e:
            print(f"LCD command error: {e}")

    def write_data(self, data):
        """Invia dati al display"""
        self._set_channel()
        try:
            # Dati su entrambi i nibble (4-bit mode)
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x01, (data & 0xF0) | 0x05)
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x01, (data & 0xF0) | 0x01)
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x01, ((data << 4) & 0xF0) | 0x05)
            self.bus.write_byte_data(LCD_I2C_ADDR, 0x01, ((data << 4) & 0xF0) | 0x01)
            time.sleep(0.0001)
        except Exception as e:
            print(f"LCD data error: {e}")

    def init_lcd(self):
        """Inizializzazione display LCD"""
        self._set_channel()
        time.sleep(0.05)
        
        # Sequenza di inizializzazione
        self.write_command(0x33)
        self.write_command(0x32)
        self.write_command(0x28)  # 4-bit mode, 2 line
        self.write_command(0x0C)  # Display on, cursor off
        self.write_command(0x06)  # Increment cursor
        self.write_command(0x01)  # Clear display
        time.sleep(0.002)

    def set_backlight(self, on=True):
        """Controllo retroilluminazione"""
        self.backlight = on
        self._set_channel()
        try:
            state = 0x01 if on else 0x00
            self.bus.write_byte(LCD_I2C_ADDR, state << 3)
        except:
            pass

    def write_string(self, text, line=0):
        """Scrivi una stringa su una riga specifica"""
        if line == 0:
            self.write_command(0x80)
        elif line == 1:
            self.write_command(0xC0)
        
        for char in text:
            self.write_data(ord(char))

# Istanza globale
lcd_controller = LCDController()

def initialize():
    try:
        lcd_controller.init_lcd()
        lcd_controller.set_backlight(True)
        time.sleep(0.1)
        set_status("SERVER START")
    except Exception as e:
        print(f"LCD init error: {e}")

def set_status(msg: str):
    with state.lcd_lock:
        try:
            lcd_controller.write_string(msg.ljust(16), 0)
        except Exception as e:
            print(f"Status error: {e}")

def set_speed(sp: int):
    with state.lcd_lock:
        try:
            txt = f"Spd:{sp}%".ljust(16)
            lcd_controller.write_string(txt, 1)
        except Exception as e:
            print(f"Speed error: {e}")

def lcd_handler():
    while True:
        with state.lcd_lock:
            if state.pico and state.pico.is_open:
                try:
                    line = state.pico.readline().decode().strip()
                    if line.startswith("STATUS|"):
                        set_status(line.split("|",1)[1][:16])
                    elif line.startswith("SPEED|"):
                        try:
                            sp = int(line.split("|",1)[1])
                            set_speed(sp)
                        except: 
                            pass
                except Exception as e:
                    print(f"LCD read error: {e}")
        time.sleep(0.05)