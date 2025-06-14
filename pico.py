from machine import Pin, PWM, I2C, SoftI2C
import time
import sys
import ujson
import _thread

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

# --- Funzioni lettura encoder (come primo esempio) ---
def read_encoder_raw(i2c_bus):
    try:
        buf = i2c_bus.readfrom_mem(ENCODER_ADDR, 0x0C, 2)
        raw = (buf[0] << 8) | buf[1]
        return raw & 0x0FFF
    except Exception as e:
        print("I2C ERROR:", e)
        return 0

def read_encoder_angle(i2c_bus):
    raw = read_encoder_raw(i2c_bus)
    angle = raw * 360.0 / 4096.0
    return angle

# --- Comunicazione ---
def send_status(msg):
    sys.stdout.write(f"STATUS|{msg}\n")

def send_speed(sp):
    sys.stdout.write(f"SPEED|{sp}\n")

def read_encoders():
    angles = {
        'j1': read_encoder_angle(encoders['BASE']),
        'j2': read_encoder_angle(encoders['M1']),
        'j3': read_encoder_angle(encoders['M2'])
    }
    sys.stdout.write(f"ANG|{ujson.dumps(angles)}\n")
    return angles

def send_position():
    sys.stdout.write(f"POS|{ujson.dumps(current_position)}\n")

# --- Movimento a passi ---
def do_steps(axis, steps, speed_pct, jog=False):
    global alarm, current_speed
    if alarm:
        send_status("ALARM BLOCK")
        return
        
    mm = steps / STEP_PER_MM
    current_speed = speed_pct
    send_speed(speed_pct)
    
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
    send_status(f"MOVE {axis} {mm:.2f}mm")

# --- Controllo gripper ---
def move_servo(angle):
    global alarm
    if alarm:
        send_status("ALARM BLOCK")
        return
        
    angle = max(0, min(180, angle))
    duty = int(1638 + (angle/180)*(8192-1638))
    servo.duty_u16(duty)
    send_status(f"GRIP {angle}°")

# --- Thread lettura angoli ---
last_angles = {}
def send_angles_periodically():
    while True:
        current_angles = {
            'j1': read_encoder_angle(encoders['BASE'])#,
            #'j2': read_encoder_angle(encoders['M1']),
            #'j3': read_encoder_angle(encoders['M2'])
        }
        
        if current_angles != last_angles:
            sys.stdout.write(f"ANG|{ujson.dumps(current_angles)}\n")
            last_angles.update(current_angles)
            
        time.sleep(0.1)  # 20 letture/secondo

# --- Homing semplificato ---
def homing():
    global alarm
    if alarm:
        send_status("ALARM BLOCK")
        return
        
    send_status("HOMING START")
    delay = 0.002 * (100 - HOMING_SPEED) / 100
    
    for ax, target in HOME_ANGLES.items():
        drv = drivers[ax]
        drv['dir'].value(HOMING_DIR[ax])
        
        while True:
            angle = read_encoder_angle(encoders[ax])
            if (HOMING_DIR[ax] and angle >= target) or (not HOMING_DIR[ax] and angle <= target):
                break
                
            drv['step'].value(1)
            time.sleep(delay)
            drv['step'].value(0)
            time.sleep(delay)
            
        send_status(f"HOMED {ax} {target}°")
    
    read_encoders()
    send_status("HOMING DONE")

# --- Loop principale ---
def main():
    # Scansione iniziale I2C
    print("I2C scan BASE:", encoders['BASE'].scan())
    print("I2C scan M1:", encoders['M1'].scan())
    print("I2C scan M2:", encoders['M2'].scan())
    
    send_status("INIT COMPLETE")
    print("PICO READY")
    
    # Avvio thread lettura angoli
    _thread.start_new_thread(send_angles_periodically, ())
    
    global alarm
    while True:
        line = sys.stdin.readline().strip()
        if not line: 
            continue
            
        try:
            cmd = ujson.loads(line)
        except Exception as e:
            send_status(f"ERROR CMD: {str(e)}")
            continue
            
        c = cmd.get('cmd')
        if c == 'stop':
            alarm = True
            led.value(1)
            send_status("EMERGENCY STOP!")
        elif c == 'reset':
            alarm = False
            led.value(0)
            send_status("RESET OK")
        elif c == 'homing':
            homing()
        elif c == 'getenc':
            read_encoders()
        elif c == 'getpos':
            send_position()
        elif c == 'speed':
            val = cmd.get('speed_pct', current_speed)
            if 5 <= val <= 100:
                current_speed = val
                send_speed(val)
            else:
                send_status("INVALID SPEED")
        elif cmd.get('axis') in drivers and 'mm' in cmd:
            axis = cmd['axis']
            mm = float(cmd['mm'])
            sp = int(cmd.get('speed_pct', current_speed))
            do_steps(axis, int(mm * STEP_PER_MM), sp)
        elif c == 'grip':
            move_servo(cmd.get('angle', 0))
        elif c == 'reboot':
            send_status("REBOOTING")
            time.sleep(0.5)
            machine.reset()
        else:
            send_status(f"UNKNOWN CMD: {c}")

if __name__ == '__main__':
    main()

