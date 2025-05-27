import sys, ujson, time
from machine import Pin, PWM, I2C, reset
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

# --- Custom exception per interruzione immediata ---
class EmergencyStop(Exception):
    pass

# --- CONFIG PIN & PARAMS ---
HOMING_DIR       = {'BASE':0, 'M1':0, 'M2':0}
HOME_STEPS       = {'BASE':100, 'M1':1000, 'M2':1000}
HOMING_SPEED_PCT = 10    # Velocità (%) durante la sequenza di homing

drivers      = {
  'BASE':{'step':Pin(2,Pin.OUT),'dir':Pin(3,Pin.OUT)},
  'M1':  {'step':Pin(4,Pin.OUT),'dir':Pin(5,Pin.OUT)},
  'M2':  {'step':Pin(6,Pin.OUT),'dir':Pin(7,Pin.OUT)},
}
endstops     = {
  'BASE':Pin(10,Pin.IN,Pin.PULL_UP),
  'M1':  Pin(11,Pin.IN,Pin.PULL_UP),
  'M2':  Pin(12,Pin.IN,Pin.PULL_UP),
}
servo        = PWM(Pin(15)); servo.freq(50)
led          = Pin("LED",Pin.OUT)
alarm        = False
current_speed=100

# --- LCD 16×2 I2C ---
i2c      = I2C(0,sda=Pin(0),scl=Pin(1),freq=400000)
lcd_addr = i2c.scan()[0]
lcd      = I2cLcd(i2c,lcd_addr,2,16)
def lcd_log(msg):
    """
    Riga 1: ultimo messaggio di log (msg)
    Riga 2: velocità corrente (current_speed)
    """
    lcd.clear()
    lcd.putstr(msg[:16])
    lcd.move_to(0,1)
    # Mostra "Spd:XX%" e riempie con spazi a 16 colonne
    spd = f"Spd:{current_speed}%"
    if len(spd) < 16:
        spd += ' ' * (16 - len(spd))
    else:
        spd = spd[:16]
    lcd.putstr(spd)

# --- MOTION FUNCTIONS ---
def do_steps(axis, steps, speed_pct):
    global alarm, current_speed
    current_speed = speed_pct
    d = drivers[axis]
    d['dir'].value(1 if steps>=0 else 0)
    delay = max(0.0002, 0.003*(100-speed_pct)/90)
    for _ in range(abs(steps)):
        if alarm:
            raise EmergencyStop()
        d['step'].value(1); time.sleep(delay)
        d['step'].value(0); time.sleep(delay)
    lcd_log(f"{axis} move {steps//80}")

def move_servo(angle):
    global alarm
    if alarm:
        raise EmergencyStop()
    duty = int(1638 + (angle/180)*(8192-1638))
    servo.duty_u16(duty)
    lcd_log(f"Grip {angle}°")

# --- HOMING double-touch + offset at reduced speed ---
def homing_axis(ax):
    global alarm
    end  = endstops[ax]
    d    = drivers[ax]
    to   = HOMING_DIR[ax]
    back = 1 - to
    for p in (1,2):
        if alarm:
            raise EmergencyStop()
        lcd_log(f"{ax} home p{p}")
        d['dir'].value(to)
        while end.value():
            if alarm:
                raise EmergencyStop()
            d['step'].value(1); time.sleep(0.002)
            d['step'].value(0); time.sleep(0.002)
        lcd_log(f"{ax} back")
        d['dir'].value(back)
        for _ in range(20):
            if alarm:
                raise EmergencyStop()
            d['step'].value(1); time.sleep(0.002)
            d['step'].value(0); time.sleep(0.002)

def homing():
    global alarm
    current_speed=10
    lcd_log("Homing start")
    for ax in ('BASE','M1','M2'):
        homing_axis(ax)
    for ax in ('BASE','M1','M2'):
        if alarm:
            raise EmergencyStop()
        lcd_log(f"{ax} to home")
        do_steps(ax, HOME_STEPS[ax], HOMING_SPEED_PCT)
    lcd_log("Homing done")
    
# --- IDLE UNTIL COMMAND ---
lcd_log("IDLE, Await cmd")
print("PICO IDLE")

for line in sys.stdin:
    if not line: continue
    try:
        cmd = ujson.loads(line)
        c   = cmd.get('cmd')

        if c == 'stop':
            alarm = True
            led.value(1)
            lcd_log("!EMERGENCY!","Stopped")
            # interrompi immediatamente qualunque ciclo
            raise EmergencyStop()

        elif c == 'reset':
            alarm = False
            led.value(0)
            lcd_log("Reset OK")

        elif c == 'homing':
            if not alarm:
                homing()
            else:
                lcd_log("In Alarm","")

        elif c == 'grip':
            if not alarm:
                angle = cmd.get('angle',0)
                move_servo(angle)
            else:
                lcd_log("In Alarm","")

        elif c == 'reboot':
            lcd_log("Rebooting Pico","")
            print("OK")
            time.sleep(0.5)
            reset()

        elif 'axis' in cmd and 'mm' in cmd:
            if not alarm:
                ax    = cmd['axis']
                mm    = cmd['mm']
                sp    = cmd.get('speed_pct',current_speed)
                steps = int(mm * 80)
                do_steps(ax, steps, sp)
            else:
                lcd_log("In Alarm","")

        else:
            lcd_log("Unknown cmd","")

        print("OK")

    except EmergencyStop:
        # silenzia il traceback, resta in loop ma non continua il movimento
        continue
    except Exception as e:
        lcd_log("Err", str(e)[:16])
        print("ERR", e)

