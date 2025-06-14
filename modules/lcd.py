# modules/lcd.py
from RPLCD.i2c import CharLCD
import time
from .shared_state import state

LCD_I2C_ADDR = 0x27
LCD_COLUMNS = 16
LCD_ROWS = 2
I2C_PORT = 1

def initialize():
    try:
        state.lcd = CharLCD(
            i2c_expander='PCF8574',
            address=LCD_I2C_ADDR,
            port=I2C_PORT,
            cols=LCD_COLUMNS,
            rows=LCD_ROWS,
            auto_linebreaks=False
        )
    except Exception as e:
        print(f"LCD init error: {e}")
        state.lcd = None

def set_status(msg: str):
    if not state.lcd:
        return
        
    try:
        with state.lcd_lock:
            state.lcd.cursor_pos = (0, 0)
            state.lcd.write_string(msg[:LCD_COLUMNS].ljust(LCD_COLUMNS))
    except Exception as e:
        print(f"LCD write error: {e}")
        state.lcd = None

def set_speed(sp: int):
    if not state.lcd:
        return
        
    try:
        with state.lcd_lock:
            state.lcd.cursor_pos = (1, 0)
            txt = f"Spd:{sp}%".ljust(LCD_COLUMNS)
            state.lcd.write_string(txt)
    except Exception as e:
        print(f"LCD write error: {e}")
        state.lcd = None

def lcd_handler():
    while True:
        try:
            if state.pico and state.pico.is_open:
                try: 
                    # Add timeout to prevent hangs
                    line = state.pico.readline().decode().strip()
                except Exception as e:
                    print(f"LCD read error: {e}")
                    line = ""
                    
                if line.startswith("STATUS|"):
                    set_status(line.split("|",1)[1])
                elif line.startswith("SPEED|"):
                    try:
                        sp = int(line.split("|",1)[1])
                        set_speed(sp)
                    except Exception as e:
                        print(f"Speed parse error: {e}")
        except Exception as e:
            print(f"LCD handler error: {e}")
            
        time.sleep(0.1)  # Reduced polling frequency