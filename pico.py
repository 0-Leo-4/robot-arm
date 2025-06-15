from machine import Pin, PWM, I2C, SoftI2C
import time
import sys
import ujson
import _thread
import math

# --- CONFIG PIN & PARAMS ---
HOMING_DIR   = {'BASE': 0, 'M1': 0, 'M2': 1}      # 0=decrease, 1=increase
HOME_ANGLES  = {'BASE': 10.0, 'M1': 5.0, 'M2': 0.0}  # obiettivi in gradi
HOMING_SPEED = 50    # percentuale
STEP_PER_MM  = 160   # passi per mm
ENCODER_ADDR = 0x36  # Indirizzo fisso AS5600

# Driver step/dir
drivers = {
    'BASE': {'step': Pin(2, Pin.OUT), 'dir': Pin(3, Pin.OUT)},
    'M1':   {'step': Pin(4, Pin.OUT), 'dir': Pin(5, Pin.OUT)},
    'M2':   {'step': Pin(6, Pin.OUT), 'dir': Pin(7, Pin.OUT)},
}

# Servo gripper
servo = PWM(Pin(15))
servo.freq(50)

# LED onboard
led = Pin("LED", Pin.OUT)

# Sensori AS5600 su I2C
i2c0 = I2C(0, scl=Pin(9),  sda=Pin(8),  freq=400_000)
i2c1 = I2C(1, scl=Pin(11), sda=Pin(10), freq=400_000)
i2c2 = SoftI2C(scl=Pin(13), sda=Pin(14), freq=400_000)

# Dizionario bus I2C per ogni asse
encoders = {
    'BASE': i2c0,
    'M1':   i2c1,
    'M2':   i2c2
}

# Stato globale
alarm = False
current_speed = 100
current_position = {'BASE': 0.0, 'M1': 0.0, 'M2': 0.0}  # in mm

# --- Funzioni lettura encoder robuste ---
def read_encoder_raw(i2c_bus, max_retries=3):
    for attempt in range(max_retries):
        try:
            buf = i2c_bus.readfrom_mem(ENCODER_ADDR, 0x0C, 2)
            raw = (buf[0] << 8) | buf[1]
            return raw & 0x0FFF
        except Exception as e:
            if attempt == max_retries - 1:
                print(f"I2C ERROR (final): {e}")
                return 0
            time.sleep(0.005)  # Breve pausa prima di riprovare

def read_encoder_angle(i2c_bus):
    raw = read_encoder_raw(i2c_bus)
    if raw == 0:  # Valore di fallback
        return 0.0
    return raw * 360.0 / 4096.0

# --- Comunicazione robusta ---
def safe_send_data(data_str):
    try:
        sys.stdout.write(data_str + "\n")
    except Exception as e:
        print(f"Serial write error: {e}")

def send_all_data():
    """Invia tutti i dati in un unico messaggio con gestione errori"""
    try:
        angles = {
            'j1': read_encoder_angle(encoders['BASE']),
            'j2': read_encoder_angle(encoders['M1']),
            'j3': read_encoder_angle(encoders['M2'])
        }
        data = {'angles': angles, 'position': current_position}
        safe_send_data(f"ALL|{ujson.dumps(data)}")
    except Exception as e:
        print(f"Error in send_all_data: {e}")

# --- Movimento a passi ---
def do_steps(axis, steps, speed_pct, jog=False):
    global alarm, current_speed
    if alarm:
        safe_send_data("STATUS|ALARM BLOCK")
        return
        
    mm = steps / STEP_PER_MM
    current_speed = speed_pct
    safe_send_data(f"SPEED|{speed_pct}")
    
    drv = drivers[axis]
    direction = 1 if steps >= 0 else 0
    steps = abs(steps)
    
    drv['dir'].value(direction)
    step_delay = max(0.0005, 0.001 * (100 - speed_pct) / 100)
    
    for _ in range(steps):
        drv['step'].value(1)
        time.sleep(step_delay)
        drv['step'].value(0)
        time.sleep(step_delay)
        
    current_position[axis] += (mm if direction else -mm)
    safe_send_data(f"STATUS|MOVE {axis} {mm:.2f}mm")

# --- Controllo gripper ---
def move_servo(angle):
    global alarm
    if alarm:
        safe_send_data("STATUS|ALARM BLOCK")
        return
        
    angle = max(0, min(180, angle))
    duty = int(1638 + (angle/180)*(8192-1638))
    servo.duty_u16(duty)
    safe_send_data(f"STATUS|GRIP {angle}°")

# --- Thread lettura angoli (ridotta frequenza) ---
def send_angles_periodically():
    while True:
        try:
            angles = {
                'j1': read_encoder_angle(encoders['BASE']),
                'j2': read_encoder_angle(encoders['M1']),
                'j3': read_encoder_angle(encoders['M2'])
            }
            safe_send_data(f"ANG|{ujson.dumps(angles)}")
        except Exception as e:
            print(f"Angle thread error: {e}")
        time.sleep(0.5)  # Aggiornamento più lento (2Hz)

# --- Homing semplificato ---
def homing():
    global alarm
    if alarm:
        safe_send_data("STATUS|ALARM BLOCK")
        return
        
    safe_send_data("STATUS|HOMING START")
    delay = 0.002 * (100 - HOMING_SPEED) / 100
    
    for ax, target in HOME_ANGLES.items():
        drv = drivers[ax]
        drv['dir'].value(HOMING_DIR[ax])
        
        while True:
            try:
                angle = read_encoder_angle(encoders[ax])
                if (HOMING_DIR[ax] and angle >= target) or (not HOMING_DIR[ax] and angle <= target):
                    break
                    
                drv['step'].value(1)
                time.sleep(delay)
                drv['step'].value(0)
                time.sleep(delay)
            except Exception as e:
                print(f"Homing error: {e}")
                break
            
        safe_send_data(f"STATUS|HOMED {ax} {target}°")
    
    send_all_data()
    safe_send_data("STATUS|HOMING DONE")

# --- Loop principale robusto ---
def main():
    # Scansione iniziale I2C
    print("I2C scan BASE:", encoders['BASE'].scan())
    print("I2C scan M1:", encoders['M1'].scan())
    print("I2C scan M2:", encoders['M2'].scan())
    
    safe_send_data("STATUS|INIT COMPLETE")
    print("PICO READY")
    
    # Avvio thread lettura angoli (a frequenza ridotta)
    _thread.start_new_thread(send_angles_periodically, ())
    
    global alarm
    while True:
        try:
            line = sys.stdin.readline().strip()
            if not line: 
                continue
                
            cmd = ujson.loads(line)
            c = cmd.get('cmd')
            
            if c == 'stop':
                alarm = True
                led.value(1)
                safe_send_data("STATUS|EMERGENCY STOP!")
            elif c == 'reset':
                alarm = False
                led.value(0)
                safe_send_data("STATUS|RESET OK")
            elif c == 'homing':
                homing()
            elif c == 'getall':
                send_all_data()
            elif c == 'getenc':
                # Mantenuto per compatibilità
                angles = {
                    'j1': read_encoder_angle(encoders['BASE']),
                    'j2': read_encoder_angle(encoders['M1']),
                    'j3': read_encoder_angle(encoders['M2'])
                }
                safe_send_data(f"ANG|{ujson.dumps(angles)}")
            elif c == 'getpos':
                # Mantenuto per compatibilità
                safe_send_data(f"POS|{ujson.dumps(current_position)}")
            elif c == 'speed':
                val = cmd.get('speed_pct', current_speed)
                if 5 <= val <= 100:
                    current_speed = val
                    safe_send_data(f"SPEED|{val}")
                else:
                    safe_send_data("STATUS|INVALID SPEED")
            elif cmd.get('axis') in drivers and 'mm' in cmd:
                axis = cmd['axis']
                mm = float(cmd['mm'])
                sp = int(cmd.get('speed_pct', current_speed))
                do_steps(axis, int(mm * STEP_PER_MM), sp)
            elif c == 'grip':
                move_servo(cmd.get('angle', 0))
            elif c == 'reboot':
                safe_send_data("STATUS|REBOOTING")
                time.sleep(0.5)
                machine.reset()
            else:
                safe_send_data(f"STATUS|UNKNOWN CMD: {c}")
                
        except ujson.JSONDecodeError:
            print(f"Invalid JSON: {line}")
        except Exception as e:
            print(f"Main loop error: {e}")
            # Tentativo di ripristino
            try:
                sys.stdin.flush()
            except:
                pass
            time.sleep(0.1)

if __name__ == '__main__':
    main()
