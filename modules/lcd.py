# modules/lcd.py
import time
import smbus2
import threading
from .shared_state import state

# Configurazione hardware
TCA9548A_ADDR = 0x70  # Indirizzo multiplexer I2C
LCD_I2C_ADDR = 0x27    # Indirizzo LCD I2C
LCD_CHANNEL = 0x08     # Canale 4 del multiplexer (0x08 = bit 3)
I2C_BUS = 1            # Bus I2C 1 (GPIO 2/3)

# Comandi LCD
LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_FUNCTIONSET = 0x20
LCD_SETDDRAMADDR = 0x80

# Flags per display control
LCD_DISPLAYON = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON = 0x02
LCD_CURSOROFF = 0x00
LCD_BLINKON = 0x01
LCD_BLINKOFF = 0x00

# Flags per entry mode
LCD_ENTRYRIGHT = 0x00
LCD_ENTRYLEFT = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00

# Flags per function set
LCD_4BITMODE = 0x00
LCD_2LINE = 0x08
LCD_5x8DOTS = 0x00

# Pin mapping per PCF8574
LCD_ENABLE_BIT = 0x04  # Bit 2: Enable
LCD_BACKLIGHT_BIT = 0x08  # Bit 3: Backlight
LCD_RS_BIT = 0x01      # Bit 0: Register Select (0=command, 1=data)

class LCDController:
    def __init__(self):
        self.bus = smbus2.SMBus(I2C_BUS)
        self.backlight = True
        self.initialized = False
        self.lock = threading.Lock()
        
    def _set_channel(self):
        """Seleziona il canale LCD sul multiplexer"""
        try:
            with self.lock:
                self.bus.write_byte(TCA9548A_ADDR, LCD_CHANNEL)
            time.sleep(0.002)  # Tempo di stabilizzazione
            return True
        except Exception as e:
            print(f"LCD channel error: {e}")
            return False
            
    def _write_byte(self, value, mode=0):
        """Scrive un byte al display in modalità 4-bit"""
        if not self._set_channel():
            return False
            
        try:
            # Divide il byte in due nibble (4-bit)
            high_nibble = value & 0xF0
            low_nibble = (value << 4) & 0xF0
            
            # Preparazione dati con flag
            data_high = high_nibble | mode | LCD_BACKLIGHT_BIT if self.backlight else high_nibble | mode
            data_low = low_nibble | mode | LCD_BACKLIGHT_BIT if self.backlight else low_nibble | mode
            
            # Invia nibble alto con enable pulse
            with self.lock:
                self.bus.write_byte(LCD_I2C_ADDR, data_high | LCD_ENABLE_BIT)
                self.bus.write_byte(LCD_I2C_ADDR, data_high)
            
            # Invia nibble basso con enable pulse
            with self.lock:
                self.bus.write_byte(LCD_I2C_ADDR, data_low | LCD_ENABLE_BIT)
                self.bus.write_byte(LCD_I2C_ADDR, data_low)
            
            time.sleep(0.0001)  # Attesa minima tra i comandi
            return True
        except Exception as e:
            print(f"LCD write error: {e}")
            return False

    def command(self, cmd):
        """Invia un comando al display"""
        return self._write_byte(cmd, 0)

    def write_data(self, data):
        """Invia dati al display"""
        return self._write_byte(data, LCD_RS_BIT)

    def init_lcd(self):
        """Inizializzazione display LCD"""
        if not self._set_channel():
            return False
            
        try:
            # Sequenza di inizializzazione per 4-bit mode
            time.sleep(0.05)  # Attesa dopo power-on
            
            # Sequenza di reset (3 volte)
            for _ in range(3):
                self._write_byte(0x30, 0)
                time.sleep(0.005)  # Attesa lunga per reset
            
            # Imposta 4-bit mode
            self._write_byte(0x20, 0)
            time.sleep(0.001)
            
            # Configurazione display: 2 linee, carattere 5x8
            self.command(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS)
            time.sleep(0.001)
            
            # Display on, cursore off, blink off
            self.command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF)
            time.sleep(0.001)
            
            # Modalità entry: incrementa cursore, no shift
            self.command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT)
            time.sleep(0.001)
            
            # Pulisci display
            self.clear()
            
            self.initialized = True
            return True
        except Exception as e:
            print(f"LCD init error: {e}")
            return False

    def clear(self):
        """Pulisce il display e porta il cursore a home"""
        self.command(LCD_CLEARDISPLAY)
        time.sleep(0.002)  # Il comando clear richiede più tempo

    def set_backlight(self, on=True):
        """Controlla la retroilluminazione"""
        self.backlight = on
        # Aggiorna lo stato inviando un comando dummy
        self._write_byte(0, 0)

    def write_string(self, text, line=0):
        """Scrivi una stringa su una riga specifica"""
        if not self.initialized:
            return
            
        # Imposta l'indirizzo DDRAM per la riga richiesta
        if line == 0:
            self.command(LCD_SETDDRAMADDR | 0x00)  # Prima riga
        elif line == 1:
            self.command(LCD_SETDDRAMADDR | 0x40)  # Seconda riga
        
        # Invia i caratteri uno per uno
        for char in text:
            self.write_data(ord(char))

# Istanza globale
lcd_controller = LCDController()

def initialize():
    """Inizializza il display LCD"""
    print("Inizializzazione LCD...")
    if lcd_controller.init_lcd():
        lcd_controller.set_backlight(True)
        set_status("SERVER START")
        print("LCD inizializzato con successo")
        return True
    
    print("Fallito inizializzazione LCD")
    return False

def set_status(msg: str):
    """Imposta il messaggio di stato sulla prima riga"""
    if not lcd_controller.initialized:
        return
        
    try:
        # Tronca il messaggio a 16 caratteri e riempi con spazi
        msg = msg[:16].ljust(16)
        lcd_controller.write_string(msg, 0)
    except Exception as e:
        print(f"Status error: {e}")

def set_speed(sp: int):
    """Imposta la velocità sulla seconda riga"""
    if not lcd_controller.initialized:
        return
        
    try:
        txt = f"Spd:{sp}%".ljust(16)
        lcd_controller.write_string(txt, 1)
    except Exception as e:
        print(f"Speed error: {e}")

def lcd_handler():
    """Gestisce gli aggiornamenti LCD dai messaggi Pico"""
    while True:
        if state.pico and state.pico.is_open:
            try:
                # Leggi una linea dalla seriale (non bloccante)
                line = state.pico.readline().decode().strip()
                if not line:
                    time.sleep(0.01)
                    continue
                    
                if line.startswith("STATUS|"):
                    set_status(line.split("|",1)[1])
                elif line.startswith("SPEED|"):
                    try:
                        sp = int(line.split("|",1)[1])
                        set_speed(sp)
                    except ValueError:
                        pass
            except Exception as e:
                print(f"LCD read error: {e}")
        
        time.sleep(0.05)  # Ciclo a 20Hz