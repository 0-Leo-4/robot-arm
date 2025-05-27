from flask import Flask, request, jsonify, render_template
import threading, serial, time, json, sys, os, math
import smbus2
from i2c_lcd.i2c_lcd import I2cLcd

app = Flask(__name__, template_folder='templates', static_folder='static')

# —————————————————————
#  CONFIGURAZIONE GENERALE
# —————————————————————
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE    = 115200
LCD_I2C_ADDR = 0x27

# stato globale
emergency_active = False
current_speed    = 100
current_steps    = [0,0,0]
command_queue    = []
lock             = threading.Lock()

# —————————————————————
#  INIZIALIZZA LCD I²C
# —————————————————————
i2c_bus = smbus2.SMBus(1)
lcd = I2cLcd(i2c_bus, LCD_I2C_ADDR, rows=2, cols=16)
def lcd_clear():
    lcd.clear()
def lcd_status(msg):
    lcd.clear()
    lcd.putstr(msg[:16])
def lcd_speed(sp):
    lcd.move_to(0,1)
    lcd.putstr(f"Spd:{sp}%".ljust(16))

# —————————————————————
#  APRE/RIAPRE LA SERIAL AL PICO
# —————————————————————
pico = None
def open_pico():
    global pico
    try:
        serial_port = SERIAL_PORT if not sys.platform.startswith('win') else 'COM6'
        p = serial.Serial(serial_port, BAUDRATE, timeout=0.1, write_timeout=0.1)
        time.sleep(2)
        p.reset_input_buffer()
        p.reset_output_buffer()
        pico = p
    except Exception:
        pico = None

open_pico()

# thread di monitor per disconnessioni
def monitor_pico():
    global pico
    while True:
        if pico:
            try:
                _ = pico.in_waiting
            except:
                pico.close()
                pico = None
        time.sleep(1)

threading.Thread(target=monitor_pico, daemon=True).start()

# —————————————————————
#  HANDLER SERIALE → LCD
# —————————————————————
def lcd_handler():
    """Legge dal Pico via seriale e aggiorna l’I²C-LCD."""
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue
        if line.startswith("STATUS|"):
            msg = line.split("|",1)[1]
            lcd.clear()
            lcd.putstr(msg[:16])
        elif line.startswith("SPEED|"):
            sp = line.split("|",1)[1]
            lcd.move_to(0,1)
            lcd.putstr(f"Spd:{sp}%".ljust(16))
        # piccolo delay per non saturare la CPU
        time.sleep(0.01)

# Avvia il thread alla fine delle dichiarazioni: 
threading.Thread(target=lcd_handler, daemon=True).start()

# wrapper scrittura Pico
def try_write(cmd):
    global pico
    with lock:
        if not pico or not pico.is_open:
            return
        try:
            pico.reset_output_buffer()
            pico.write((json.dumps(cmd)+"\n").encode())
            pico.flush()
        except:
            pico.close()

# —————————————————————
#  FLASK ROUTES
# —————————————————————

@app.route('/')
def index():
    return render_template('control.html')

@app.route('/api/pico_status')
def pico_status():
    if not pico or not pico.is_open:
        open_pico()
    ok = pico and pico.is_open
    return jsonify(connected=ok)

@app.route('/api/disconnect_pico', methods=['POST'])
def disconnect_pico():
    global pico
    if pico:
        pico.close()
    return jsonify(status='disconnected')

@app.route('/api/reconnect_pico', methods=['POST'])
def reconnect_pico():
    open_pico()
    return jsonify(status='reconnected' if pico else 'error')

@app.route('/api/set_speed', methods=['POST'])
def set_speed():
    global current_speed
    v = request.json.get('speed_pct',100)
    current_speed = v
    lcd_speed(v)
    try_write({"cmd":"speed","speed_pct":v})
    return jsonify(status='ok')

@app.route('/api/jog', methods=['POST'])
def jog():
    if emergency_active:
        return jsonify(status='blocked'), 403
    d = request.json
    try_write({"axis": d['axis'], "mm": d['mm'], "speed_pct": current_speed})
    return jsonify(status='ok')

@app.route('/api/homing', methods=['POST'])
def homing():
    if emergency_active:
        return jsonify(status='blocked'), 403
    try_write({"cmd":"homing"})
    return jsonify(status='ok')

@app.route('/api/stop', methods=['POST'])
def stop():
    global emergency_active
    emergency_active = True
    try_write({"cmd":"stop"})
    return jsonify(status='stopped')

@app.route('/api/reset', methods=['POST'])
def reset_alarm():
    global emergency_active
    emergency_active = False
    try_write({"cmd":"reset"})
    return jsonify(status='reset')

# in futuro: /upload, /run_queue, /move_point…

if __name__ == '__main__':
    lcd_status("SERVER START")
    app.run(host='0.0.0.0', port=5000, threaded=True)
