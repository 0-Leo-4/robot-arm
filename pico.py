from machine import Pin, PWM
import time
import sys
import ujson
import machine

# --- CONFIG PIN & PARAMS ---
STEP_PER_MM  = 160   # passi per mm

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

# Stato globale
alarm = False
current_speed = 100
current_position = {'BASE': 0.0, 'M1': 0.0, 'M2': 0.0}  # in mm

def safe_send_data(data_str):
    try:
        sys.stdout.write(data_str + "\n")
    except Exception as e:
        print(f"Serial write error: {e}")

# --- Movimento a passi ---
def do_steps(axis, steps, speed_pct):
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

# --- Loop principale robusto ---
def main():
    safe_send_data("STATUS|PICO READY")
    print("PICO READY")
    
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
                # Homing non supportato sul Pico in questa versione
                safe_send_data("STATUS|HOMING NOT SUPPORTED")
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
            time.sleep(0.1)

if __name__ == '__main__':
    main()