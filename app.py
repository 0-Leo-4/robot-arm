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

AXIS_MAP = {'X': 'BASE', 'Y': 'M1', 'Z': 'M2'}
STEP_PER_MM = 160

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

# Variabili per calibrazione visione
CALIBRATION_MATRIX = None
CALIBRATION_POINTS = []  # [(robot_x, robot_y, img_x, img_y)]
CALIBRATION_ACTIVE = False
CALIBRATION_OBJECT = None

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

def get_current_robot_position():
    """Ottieni la posizione corrente dal Pico"""
    global pico
    if not pico or not pico.is_open:
        return None
    
    try:
        try_write({"cmd": "getpos"})
        time.sleep(0.1)
        if pico.in_waiting:
            response = pico.readline().decode().strip()
            if response.startswith("POS|"):
                return json.loads(response[4:])
    except Exception as e:
        app.logger.error("Errore lettura posizione: %s", str(e))
    return None

def vision_to_robot_coordinates(img_x, img_y):
    """Converti coordinate immagine a coordinate robot"""
    global CALIBRATION_MATRIX
    
    if CALIBRATION_MATRIX is None:
        raise ValueError("Calibrazione non completata")
    
    # Converti le coordinate usando la matrice di trasformazione
    src_point = np.array([[img_x, img_y]], dtype=np.float32)
    dst_point = cv2.perspectiveTransform(src_point.reshape(1, -1, 2), np.array(CALIBRATION_MATRIX))
    return dst_point[0][0][0], dst_point[0][0][1]

def parse_gcode_to_moves(code_block, mode='absolute'):
    """
    Converte un blocco di G0/G1 in una lista di comandi 
    {'axis':'BASE'|'M1'|'M2','mm':delta,'speed_pct':...}
    usando una DDA su 3 assi per il moto rettilineo.
    """
    AXIS_MAP = {'X': 'BASE', 'Y': 'M1', 'Z': 'M2'}
    # Posizione attuale in mm
    pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
    moves = []

    for line in code_block.splitlines():
        line = line.split(';',1)[0].strip()
        m = re.match(r'^(G0|G1)\s*(.*)', line, re.IGNORECASE)
        if not m: continue
        # Estrai nuovi target
        coords = {}
        for letter, num in re.findall(r'([XYZF])([-+]?[0-9]*\.?[0-9]+)', m.group(2), re.IGNORECASE):
            if letter.upper() in ('X','Y','Z'):
                coords[letter.upper()] = float(num)
            # Ignoriamo F qui

        # Calcola deltas in mm
        deltas = {}
        for axis in ('X','Y','Z'):
            if axis in coords:
                tgt = coords[axis]
                if mode=='absolute':
                    delta = tgt - pos[axis]
                    pos[axis] = tgt
                else:
                    delta = coords[axis]
                    pos[axis] += delta
                deltas[axis] = delta

        if not deltas:
            continue

        # Numero di passi (interi) su ciascun asse
        steps = {axis:int(abs(deltas[axis])*STEP_PER_MM) for axis in deltas}
        # Massimo di passi totali → controllerà il numero di cicli
        max_steps = max(steps.values())

        # Contatori Bresenham
        err = {axis:0 for axis in steps}

        # Per ogni “tacca” del passo
        for i in range(max_steps):
            for axis, n_steps in steps.items():
                # L’errore accumulato
                err[axis] += n_steps
                if err[axis] >= max_steps:
                    # dobbiamo fare uno step su questo asse
                    mapped = AXIS_MAP[axis]
                    # direzione la impostiamo in Pico, 
                    # quindi inviamo mm = segno * (1/STEP_PER_MM)
                    sign = 1 if deltas[axis] >= 0 else -1
                    moves.append({
                        "axis": mapped,
                        "mm": sign * (1.0 / STEP_PER_MM),
                        "speed_pct": current_speed
                    })
                    err[axis] -= max_steps
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
    global detections, fps, latest_frame, CALIBRATION_OBJECT
    cap = cv2.VideoCapture(0)
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            continue

        # Converti in scala di grigi
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Inverti l'immagine per rendere i cerchi neri "luminosi" per HoughCircles
        inverted = cv2.bitwise_not(gray)

        # Applica una soglia per evidenziare solo i neri su sfondo bianco
        _, thresh = cv2.threshold(inverted, 200, 255, cv2.THRESH_BINARY)

        # Sfocatura per ridurre il rumore
        blurred = cv2.GaussianBlur(thresh, (9, 9), 2)

        # Trova i cerchi (neri su bianco)
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

        # Aggiorna il frame per lo streaming
        ret2, jpeg = cv2.imencode('.jpg', frame)
        if ret2:
            latest_frame = jpeg.tobytes()

        # Raccogli le nuove rilevazioni
        curr = []
        if circles is not None:
            for x, y, r in circles[0]:
                # Controlla che il centro sia effettivamente nero nell'immagine originale
                cx, cy = int(round(x)), int(round(y))
                if 0 <= cx < gray.shape[1] and 0 <= cy < gray.shape[0]:
                    # Considera "nero" se il valore di grigio è basso
                    if gray[cy, cx] < 60:
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
    raw = data.get('axis','').upper()
    mapped = AXIS_MAP.get(raw)
    if not mapped:
        return jsonify(status='error', error=f"Axis {raw} non valido"), 400

    try_write({
        "axis": mapped,
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
    payload = request.json or {}
    code = payload.get('code', '')

    # parse rettilineo con DDA e mapping
    moves = []
    pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}

    for line in code.splitlines():
        line = line.split(';',1)[0].strip()
        m = re.match(r'^(G0|G1)\s*(.*)', line, re.IGNORECASE)
        if not m:
            continue

        # estrai target
        coords = { letter.upper(): float(val) 
                   for letter,val in re.findall(r'([XYZ])([-+]?[0-9]*\.?[0-9]+)', m.group(2)) }

        # calcola deltas e aggiorna pos
        deltas = {}
        for ax in ('X','Y','Z'):
            if ax in coords:
                tgt = coords[ax]
                delta = tgt - pos[ax]
                pos[ax] = tgt
                deltas[ax] = delta

        if not deltas:
            continue

        # passi interi per asse
        steps = {ax: int(abs(deltas[ax])*STEP_PER_MM) for ax in deltas}
        max_steps = max(steps.values())
        err = {ax: 0 for ax in steps}

        # DDA Bresenham
        for _ in range(max_steps):
            for ax, n in steps.items():
                err[ax] += n
                if err[ax] >= max_steps:
                    sign = 1 if deltas[ax] >= 0 else -1
                    moves.append({
                        "axis": AXIS_MAP[ax],
                        "mm":   sign*(1.0/STEP_PER_MM),
                        "speed_pct": current_speed
                    })
                    err[ax] -= max_steps

    # invia tutti i moves al Pico
    for mv in moves:
        try_write(mv)
        time.sleep(0.02)
    return jsonify(status='ok', sent=len(moves))

# —————————————————————
#  VISION INTEGRATION
# —————————————————————

@app.route('/api/start_calibration', methods=['POST'])
def start_calibration():
    """Avvia una nuova sessione di calibrazione"""
    global CALIBRATION_POINTS, CALIBRATION_ACTIVE
    CALIBRATION_POINTS = []
    CALIBRATION_ACTIVE = True
    return jsonify(status='calibration_started', message="Sposta il robot sul primo punto di calibrazione")

@app.route('/api/add_calibration_point', methods=['POST'])
def add_calibration_point():
    """Aggiungi un punto di calibrazione"""
    global CALIBRATION_POINTS, CALIBRATION_ACTIVE, CALIBRATION_OBJECT
    
    if not CALIBRATION_ACTIVE:
        return jsonify(status='error', error='Calibrazione non attiva'), 400
    
    # Trova l'oggetto più vicino al centro
    with lock:
        if not detections:
            return jsonify(status='error', error='Nessun oggetto rilevato'), 400
        
        # Trova l'oggetto più vicino al centro dell'immagine
        center_x, center_y = 320, 240  # Valori medi per risoluzione 640x480
        closest = min(detections, key=lambda d: math.hypot(d['x']-center_x, d['y']-center_y))
        CALIBRATION_OBJECT = closest['id']
        
        # Ottieni coordinate immagine
        img_x = closest['x']
        img_y = closest['y']
    
    # Ottieni coordinate robot
    robot_pos = get_current_robot_position()
    if not robot_pos:
        return jsonify(status='error', error='Impossibile leggere posizione robot'), 500
    
    robot_x = robot_pos['BASE']
    robot_y = robot_pos['M1']
    
    # Salva il punto
    CALIBRATION_POINTS.append((robot_x, robot_y, img_x, img_y))
    
    return jsonify(
        status='point_added', 
        count=len(CALIBRATION_POINTS),
        robot_x=robot_x,
        robot_y=robot_y,
        img_x=img_x,
        img_y=img_y
    )

@app.route('/api/calculate_calibration', methods=['POST'])
def calculate_calibration():
    """Calcola la matrice di trasformazione"""
    global CALIBRATION_MATRIX, CALIBRATION_POINTS, CALIBRATION_ACTIVE
    
    if not CALIBRATION_ACTIVE:
        return jsonify(status='error', error='Calibrazione non attiva'), 400
    
    if len(CALIBRATION_POINTS) < 3:
        return jsonify(status='error', error='Almeno 3 punti richiesti'), 400
    
    try:
        # Prepara i punti per OpenCV
        src_pts = []  # Immagine
        dst_pts = []  # Robot
        
        for r_x, r_y, i_x, i_y in CALIBRATION_POINTS:
            src_pts.append([i_x, i_y])
            dst_pts.append([r_x, r_y])
        
        src_pts = np.array(src_pts, dtype=np.float32)
        dst_pts = np.array(dst_pts, dtype=np.float32)
        
        # Calcola la matrice di omografia
        M, _ = cv2.findHomography(src_pts, dst_pts)
        CALIBRATION_MATRIX = M.tolist()
        CALIBRATION_ACTIVE = False
        
        return jsonify(
            status='calibration_complete', 
            matrix=CALIBRATION_MATRIX,
            points=CALIBRATION_POINTS
        )
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@app.route('/api/get_calibration_status', methods=['GET'])
def get_calibration_status():
    """Restituisce lo stato della calibrazione"""
    return jsonify(
        active=CALIBRATION_ACTIVE,
        points=CALIBRATION_POINTS,
        calibrated=CALIBRATION_MATRIX is not None
    )

@app.route('/api/move_to_object', methods=['POST'])
def move_to_object():
    """Muovi il robot verso un oggetto rilevato"""
    if emergency_active:
        return jsonify(status='blocked'), 403
    
    data = request.json
    obj_id = data.get('object_id')
    
    # Trova l'oggetto nelle rilevazioni correnti
    with lock:
        obj = next((d for d in detections if d['id'] == obj_id), None)
    
    if not obj:
        return jsonify(status='error', error='Oggetto non trovato'), 404
    
    try:
        # Converti coordinate visione → robot
        robot_x, robot_y = vision_to_robot_coordinates(obj['x'], obj['y'])
        
        # Altezza di sicurezza (da configurare)
        safe_z = 50.0  
        
        # Altezza di pick (da configurare)
        pick_z = 10.0   
        
        # 1. Sposta in sicurezza sopra l'oggetto
        move_sequence = [
            {"cmd": "move_abs", "axis": "BASE", "pos": robot_x, "speed_pct": 70},
            {"cmd": "move_abs", "axis": "M1", "pos": robot_y, "speed_pct": 70},
            {"cmd": "move_abs", "axis": "M2", "pos": safe_z, "speed_pct": 50}
        ]
        
        # 2. Scendi per il pick
        move_sequence.append({"cmd": "move_abs", "axis": "M2", "pos": pick_z, "speed_pct": 30})
        
        # 3. Chiudi gripper
        move_sequence.append({"cmd": "grip", "angle": 80})  # Angolo chiusura
        
        # 4. Risali in sicurezza
        move_sequence.append({"cmd": "move_abs", "axis": "M2", "pos": safe_z, "speed_pct": 50})
        
        # Esegui la sequenza
        for cmd in move_sequence:
            try_write(cmd)
            time.sleep(0.5)  # Breve pausa tra i movimenti
        
        return jsonify(status='pick_completed')
    
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

if __name__ == '__main__':
    lcd_status("SERVER START")
    lcd_speed(current_speed)
    app.run(host='0.0.0.0', port=5000, threaded=True)