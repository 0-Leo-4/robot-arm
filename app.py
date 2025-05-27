#!/usr/bin/env python3
import subprocess
import threading
import serial
import time
import json
import sys
import os
import base64
import math
import cv2
import numpy as np
from flask import Flask, request, jsonify, render_template, Response
import smbus2
from gpiozero import OutputDevice
from RPLCD.i2c import CharLCD

app = Flask(__name__, template_folder='templates', static_folder='static')

# —————————————————————
#  CONFIGURAZIONE GENERALE
# —————————————————————
SERIAL_PORT    = '/dev/ttyACM0'
BAUDRATE       = 115200
RELAY_GPIO = 17
relay = OutputDevice(17, active_high=False, initial_value=False)
detections = []

# LCD HD44780 via PCF8574 su I²C bus 1
LCD_I2C_ADDR   = 0x27   # modifica se diverso
LCD_COLUMNS    = 16
LCD_ROWS       = 2
I2C_PORT       = 1

# stato globale
detections = []
fps = 0.0
current_speed = 100
emergency_active = False
lock = threading.Lock()
lcd_lock = threading.Lock()
latest_frame = None

# —————————————————————
#  INIZIALIZZA LCD via RPLCD
# —————————————————————
lcd = CharLCD(
    i2c_expander = 'PCF8574',
    address      = LCD_I2C_ADDR,
    port         = I2C_PORT,
    cols         = LCD_COLUMNS,
    rows         = LCD_ROWS,
    auto_linebreaks = False
)

def lcd_status(msg: str):
    with lcd_lock:
        lcd.cursor_pos = (0, 0)
        lcd.write_string(msg[:LCD_COLUMNS].ljust(LCD_COLUMNS))

def lcd_speed(sp: int):
    with lcd_lock:
        lcd.cursor_pos = (1, 0)
        txt = f"Spd:{sp}%".ljust(LCD_COLUMNS)
        lcd.write_string(txt)

# —————————————————————
#  APRE/RICOLLEGA LA SERIAL AL PICO
# —————————————————————
# apertura seriale Pico
pico = None

def open_pico():
    global pico
    try:
        port = SERIAL_PORT if not sys.platform.startswith('win') else 'COM6'
        p = serial.Serial(port, BAUDRATE, timeout=0.1, write_timeout=0.1)
        time.sleep(2)
        p.reset_input_buffer(); p.reset_output_buffer()
        pico = p
        app.logger.info("Pico connesso su %s", port)
    except Exception as e:
        pico = None
        app.logger.warning("Impossibile aprire Pico: %s", e)

open_pico()

# monitor disconnessione Pico
def monitor_pico():
    global pico
    while True:
        if pico:
            try: _ = pico.in_waiting
            except Exception:
                pico.close(); pico = None
        time.sleep(1)
threading.Thread(target=monitor_pico, daemon=True).start()

# —————————————————————
#  THREAD DI FEEDBACK DAL PICO → LCD
# —————————————————————
def lcd_handler():
    while True:
        line = ''
        if pico and pico.is_open:
            try: line = pico.readline().decode().strip()
            except: pass
        if line.startswith("STATUS|"):
            lcd_status(line.split("|",1)[1])
        elif line.startswith("SPEED|"):
            try:
                sp = int(line.split("|",1)[1])
                lcd_speed(sp)
            except: pass
        time.sleep(0.01)
threading.Thread(target=lcd_handler, daemon=True).start()

# —————————————————————
#  INVIO COMANDI AL PICO
# —————————————————————
def try_write(cmd: dict):
    global pico
    with lock:
        if not pico or not getattr(pico, 'is_open', False): return
        try:
            pico.reset_output_buffer()
            pico.write((json.dumps(cmd)+"\n").encode()); pico.flush()
        except Exception:
            pico.close(); pico = None

def capture_and_detect():
    global detections, fps, latest_frame
    cap = cv2.VideoCapture(0)
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret: continue
        
        # Process frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9,9), 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                                 param1=50, param2=30, minRadius=10, maxRadius=100)
        
        # Always update the frame regardless of detections
        ret2, jpeg = cv2.imencode('.jpg', frame)
        if ret2:
            latest_frame = jpeg.tobytes()
            
        # Update detections
        curr = []
        if circles is not None:
            for x,y,r in np.round(circles[0]).astype(int):
                curr.append({"x":int(x),"y":int(y),"r":int(r)})
        
        with lock:
            detections = curr
            
        # Maintain ~10 FPS processing
        time.sleep(max(0, 0.1 - (time.time()-t0)))

threading.Thread(target=capture_and_detect, daemon=True).start()

# —————————————————————
#  FLASK ROUTES
# —————————————————————

@app.route('/')
def index():
    return render_template('control.html')

@app.route('/video')
def video_page():
    return render_template('video.html',
        conveyor_speed='120', resolution='1280x720',
        fps=f"{fps}", detection_interval='100 ms'
    )
    
@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if latest_frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/detections')
def api_detections():
    with lock:
        return jsonify({'frame_url':'/api/frame', 'fps':fps, 'detections':detections})

@app.route('/api/frame')
def frame():
    global latest_frame
    if not latest_frame:
        return Response(status=204)
    return Response(latest_frame, mimetype='image/jpeg')

@app.route('/api/pico_status', methods=['GET'])
def pico_status():
    if not pico or not getattr(pico, 'is_open', False):
        open_pico()
    ok = pico is not None and pico.is_open
    return jsonify(connected=ok)

@app.route('/api/disconnect_pico', methods=['POST'])
def disconnect_pico():
    global pico
    if pico:
        pico.close()
        pico = None
    return jsonify(status='disconnected')

@app.route('/api/reconnect_pico', methods=['POST'])
def reconnect_pico():
    open_pico()
    return jsonify(status='reconnected' if pico else 'error')

@app.route('/api/set_speed', methods=['POST'])
def set_speed():
    global current_speed
    if emergency_active:
        return jsonify(status='blocked'), 403
    v = int(request.json.get('speed_pct', 100))
    current_speed = v
    lcd_speed(v)
    try_write({"cmd":"speed", "speed_pct": v})
    return jsonify(status='ok')

@app.route('/api/jog', methods=['POST'])
def jog():
    if emergency_active:
        return jsonify(status='blocked'), 403
    data = request.json
    try_write({
        "axis": data['axis'],
        "mm":   data['mm'],
        "speed_pct": current_speed
    })
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
    try_write({"cmd": "stop"})
    # Imposta GPIO 17 HIGH (relè ON)
    relay.on()
    return jsonify(status='stopped')

@app.route('/api/reset', methods=['POST'])
def reset_alarm():
    global emergency_active
    emergency_active = False
    try_write({"cmd": "reset"})
    # Imposta GPIO 17 LOW (relè OFF)
    relay.off()
    return jsonify(status='reset')

@app.route('/api/gpio', methods=['POST'])
def set_gpio():
    """
    Imposta lo stato di un GPIO.
    Payload: { "pin": 17, "state": 1 }
    """
    data = request.json
    pin = int(data.get('pin', 17))
    state = int(data.get('state', 0))
    try:
        if pin == RELAY_GPIO:
            if state:
                relay.on()
            else:
                relay.off()
            return jsonify(status='ok', pin=pin, state=state)
        else:
            # Per altri pin, usa gpiozero dinamicamente
            dev = OutputDevice(pin)
            if state:
                dev.on()
            else:
                dev.off()
            return jsonify(status='ok', pin=pin, state=state)
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@app.route('/api/upload', methods=['POST'])
def upload_commands():
    """
    Riceve una lista di comandi da eseguire in coda.
    Esempio payload: {"commands":[{"cmd":"grip","angle":0}]}
    """
    if emergency_active:
        return jsonify(status='blocked'), 403
    cmds = request.json.get('commands', [])
    # Salva la coda in una variabile globale o file temporaneo
    app.config['command_queue'] = cmds
    return jsonify(status='uploaded')

@app.route('/api/run_queue', methods=['POST'])
def run_queue():
    """
    Esegue i comandi caricati tramite /api/upload.
    """
    if emergency_active:
        return jsonify(status='blocked'), 403
    cmds = app.config.get('command_queue', [])
    for cmd in cmds:
        try_write(cmd)
        time.sleep(0.05)  # breve pausa tra i comandi
    app.config['command_queue'] = []
    return jsonify(status='queue_run')

@app.route('/api/reboot_pi', methods=['POST'])
def reboot_pi():
    """Riavvia il Raspberry Pi."""
    try:
        threading.Thread(target=lambda: os.system('sudo reboot')).start()
        return jsonify(status='rebooting')
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@app.route('/api/reboot_pico', methods=['POST'])
def reboot_pico():
    """Riavvia il Pico tramite reset della seriale."""
    global pico
    try:
        if pico:
            pico.setDTR(False)
            time.sleep(0.2)
            pico.setDTR(True)
            time.sleep(0.2)
        return jsonify(status='rebooted')
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@app.route('/api/git_pull', methods=['POST'])
def git_pull():
    """
    Esegue `git pull` nella cartella corrente, restituisce stdout/stderr,
    e riavvia l'app Flask.
    """
    try:
        # Esegui git pull
        result = subprocess.run(
            ['git', 'pull'],
            cwd=os.path.dirname(os.path.abspath(__file__)),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=30
        )
        output = result.stdout + result.stderr
        status = 'ok' if result.returncode == 0 else 'error'
        # Riavvia l'app (processo Python)
        if result.returncode == 0:
            threading.Thread(target=lambda: (time.sleep(1), os._exit(0))).start()
        return jsonify(status=status, output=output)
    except Exception as e:
        return jsonify(status='error', output=str(e)), 500
    
if __name__ == '__main__':
    lcd_status("SERVER START")
    lcd_speed(current_speed)
    app.run(host='0.0.0.0', port=5000, threaded=True)
    relay.on()