#!/usr/bin/env python3
import subprocess
import threading
import serial
import time
import json
import sys
import os
import re
import base64
import math
import cv2
import numpy as np
from flask import Flask, request, jsonify, render_template, Response
import smbus2
from gpiozero import OutputDevice
from RPLCD.i2c import CharLCD
from scipy.optimize import linear_sum_assignment

app = Flask(__name__, template_folder='templates', static_folder='static')

# —————————————————————
#  CONFIGURAZIONE GENERALE
# —————————————————————
SERIAL_PORT    = '/dev/ttyACM0'
BAUDRATE       = 115200
RELAY_GPIO = 17
relay = OutputDevice(17, active_high=False, initial_value=True)
detections = []
next_circle_id = 1
tracked_circles = {}   # { id: {"x":…, "y":…, "r":…} }
max_lost_distance = 50  # px oltre i quali consideriamo sparito

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

def parse_gcode_to_moves(code_block, mode='absolute'):
    """
    Prende un blocco di G-code (solo G0/G1), restituisce lista di comandi 
    {"axis": "X"/"Y"/"Z", "mm": valore, "speed_pct": current_speed}.
    mode: 'absolute' o 'relative'
    """
    moves = []
    # manteniamo la posizione corrente
    pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
    for line in code_block.splitlines():
        line = line.split(';',1)[0].strip()  # rimuove commenti
        if not line: 
            continue
        m = re.match(r'^(G0|G1)\s*(.*)', line, re.IGNORECASE)
        if not m:
            continue
        params = m.group(2)
        # estrai parametri
        coords = {}
        for token in re.findall(r'([XYZF])([-+]?[0-9]*\.?[0-9]+)', params, re.IGNORECASE):
            letter, num = token
            coords[letter.upper()] = float(num)
        # aggiorna modalità -- opzionale, qui forziamo solo absolute
        # if 'F' in coords: ... puoi gestire feedrate
        for axis in ('X', 'Y', 'Z'):
            if axis in coords:
                target = coords[axis]
                if mode.lower() == 'relative':
                    delta = target
                else:
                    delta = target - pos[axis]
                pos[axis] += delta
                # solo se c’è spostamento
                if abs(delta) > 1e-6:
                    moves.append({
                        "axis": axis,
                        "mm": delta,
                        "speed_pct": current_speed
                    })
    return moves

def assign_ids_to_circles(new_circles):
    """
    new_circles: list di dict {"x":float, "y":float, "r":float}
    Restituisce lista di dict {"id":int, "x":…, "y":…, "r":…}
    """
    global tracked_circles, next_circle_id

    # Se non ci sono precedenti, assegna ID incrementali
    if not tracked_circles:
        results = []
        for nc in new_circles:
            cid = next_circle_id
            next_circle_id += 1
            tracked_circles[cid] = nc.copy()
            results.append({"id": cid, **nc})
        return results

    # Costruisci matrici per il cost matrix
    old_ids = list(tracked_circles.keys())
    old_pts = [tracked_circles[cid] for cid in old_ids]
    new_pts = new_circles

    cost = np.zeros((len(old_pts), len(new_pts)), dtype=float)
    for i, op in enumerate(old_pts):
        for j, np_ in enumerate(new_pts):
            cost[i, j] = math.hypot(op["x"] - np_["x"], op["y"] - np_["y"])

    # Hungarian assignment
    row_idx, col_idx = linear_sum_assignment(cost)

    assigned = {}
    results = []

    # Match existing
    for i, j in zip(row_idx, col_idx):
        if cost[i, j] < max_lost_distance:
            cid = old_ids[i]
            nc = new_pts[j]
            tracked_circles[cid] = nc.copy()
            assigned[j] = cid
            results.append({"id": cid, **nc})

    # New detections senza match → nuovi ID
    for j, nc in enumerate(new_pts):
        if j not in assigned:
            cid = next_circle_id
            next_circle_id += 1
            tracked_circles[cid] = nc.copy()
            results.append({"id": cid, **nc})

    # Rimuove i perduti: quelli vecchi non matchati
    kept_ids = {obj["id"] for obj in results}
    tracked_circles = {cid: tracked_circles[cid] for cid in kept_ids}

    return results

def capture_and_detect():
    global detections, fps, latest_frame
    cap = cv2.VideoCapture(0)
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            continue

        # Process frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=50,
            param2=30,
            minRadius=10,
            maxRadius=100
        )

        # Always update the latest frame for streaming
        ret2, jpeg = cv2.imencode('.jpg', frame)
        if ret2:
            latest_frame = jpeg.tobytes()

        # Raccogli le nuove rilevazioni
        curr = []
        if circles is not None:
            for x, y, r in circles[0]:
                curr.append({"x": float(x), "y": float(y), "r": float(r)})

        # Associa ID stabili ai cerchi
        tracked = assign_ids_to_circles(curr)

        # Calcola il delta time e aggiorna fps
        dt = time.time() - t0
        if dt > 0:
            fps = round(1.0 / dt, 1)

        # Aggiorna le detections in modo thread-safe
        with lock:
            detections = tracked

        # Mantieni il processing ad almeno ~10 FPS
        time.sleep(max(0, 0.1 - (time.time() - t0)))

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

@app.route('/api/enable_motors', methods=['POST'])
def enable_motors():
    """
    Abilita i motori attivando il relè su GPIO 17.
    """
    try:
        relay.off()
        return jsonify(status='enabled')
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

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
    
@app.route('/api/gcode', methods=['POST'])
def send_gcode():
    """
    Riceve un blocco di testo G-code, lo parsifica in movimenti 
    e li manda uno a uno al Pico.
    """
    payload = request.json or {}
    code = payload.get('code', '')
    # Parse dei movimenti (absolute mode)
    moves = parse_gcode_to_moves(code, mode='absolute')
    # Invia al Pico
    for mv in moves:
        try_write(mv)
        time.sleep(0.02)  # piccola pausa
    return jsonify(status='ok', sent=len(moves))

if __name__ == '__main__':
    lcd_status("SERVER START")
    lcd_speed(current_speed)
    app.run(host='0.0.0.0', port=5000, threaded=True)