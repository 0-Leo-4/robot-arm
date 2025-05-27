from flask import Flask, request, jsonify, render_template
import math, serial, time, json, sys, threading, os
app = Flask(__name__, template_folder='templates', static_folder='static')

#Prova
#prova 2

# --- Global state ---
emergency_active = False
current_speed = 100
current_steps = [0, 0, 0]
command_queue = []
lock = threading.Lock()

# --- Serial to Pico ---
if sys.platform.startswith('win'):
    SERIAL_PORT = 'COM6'
else:  
    SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

# Open serial
pico = None

def open_pico():
    global pico
    try:
        p = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1, write_timeout=0.1)
        time.sleep(2)
        p.reset_input_buffer()
        p.reset_output_buffer()
        app.logger.info("Pico connected")
        pico = p
    except Exception as e:
        pico = None
        app.logger.warning(f"Cannot open Pico serial: {e}")

open_pico()

def monitor_pico_disconnect():
    global pico
    while True:
        if pico is not None:
            try:
                _ = pico.in_waiting
            except Exception:
                app.logger.warning("Pico physically disconnected!")
                try:
                    pico.close()
                except:
                    pass
                pico = None
        time.sleep(1)

# Avvia il thread demone
threading.Thread(target=monitor_pico_disconnect, daemon=True).start()

# --- Robot geometry ---
L1, L2 = 139.0, 139.0
dx, dz = 55.0, -20.7
steps_per_rev = 200 * 16
steps_per_deg = steps_per_rev / 360.0

# ---- Helpers ----
def try_write(cmd):
    global pico
    line = json.dumps(cmd) + '\n'
    with lock:
        if pico is None or not getattr(pico, 'is_open', False):
            app.logger.warning("Pico non connesso: skip write")
            return
        try:
            pico.reset_output_buffer()
            pico.write(line.encode())
            pico.flush()
        except Exception as e:
            app.logger.warning(f"Serial write error → marking pico disconnected: {e}")
            try:
                pico.close()
            except:
                pass
            pico = None

def inverse_kinematics(x, z):
    dist = math.hypot(x, z)
    if dist > (L1 + L2):
        raise ValueError("Fuori portata")
    D = (x*x + z*z - L1*L1 - L2*L2)/(2*L1*L2)
    theta2 = math.acos(max(-1, min(1, D)))
    k1 = L1 + L2*math.cos(theta2)
    k2 = L2*math.sin(theta2)
    theta1 = math.atan2(z, x) - math.atan2(k2, k1)
    return math.degrees(theta1), math.degrees(theta2)

def angle_to_steps(deg):
    return int(deg * steps_per_deg)

def interpolate(csteps, tsteps):
    diff = [t - c for t, c in zip(tsteps, csteps)]
    n = max(abs(d) for d in diff)
    if n == 0:
        return []
    path = []
    for i in range(1, n + 1):
        if emergency_active:
            break
        pt = [int(c + d * i / n) for c, d in zip(csteps, diff)]
        path.append(pt)
    return path

def send_path(path):
    global current_steps
    for pts in path:
        if emergency_active:
            break
        for axis, step in zip(['BASE','M1','M2'], pts):
            if emergency_active:
                break
            cmd = {"axis": axis, "steps": step, "speed_pct": current_speed}
            try_write(cmd)
        current_steps = pts

@app.route('/video')
def video_page():
    # Qui puoi passare dati se vuoi, es: conveyor_speed=..., resolution=...
    return render_template('video.html',
        conveyor_speed='120',
        resolution='1280x720',
        fps='30',
        detection_interval='500 ms'
    )

@app.route('/api/pico_status', methods=['GET'])
def pico_status():
    global pico
    # se non è aperto, prova a riaprire
    if pico is None or not getattr(pico, 'is_open', False):
        open_pico()
    connected = pico is not None and getattr(pico, 'is_open', False)
    return jsonify(connected=connected)

@app.route('/api/disconnect_pico', methods=['POST'])
def disconnect_pico():
    global pico
    try:
        pico.close()
        return jsonify(status='pico disconnected')
    except Exception as e:
        app.logger.warning(f"Error disconnecting pico: {e}")
        return jsonify(status='error', message=str(e)), 500

@app.route('/api/reconnect_pico', methods=['POST'])
def reconnect_pico():
    open_pico()
    if pico and pico.is_open:
        return jsonify(status='pico reconnected')
    else:
        return jsonify(status='error', message='cannot reconnect'), 500
    
@app.route('/api/reboot_pi', methods=['POST'])
def reboot_pi():
    def _do_reboot():
        time.sleep(1)
        os.system('sudo reboot')
    threading.Thread(target=_do_reboot).start()
    return jsonify(status='rebooting Pi5')

@app.route('/api/reboot_pico', methods=['POST'])
def reboot_pico():
    def _do_reboot():
        global pico
        # 1) invio comando reset al Pico
        try:
            pico.reset_output_buffer()
            pico.write((json.dumps({"cmd":"reboot"}) + '\n').encode())
        except serial.SerialTimeoutException:
            app.logger.warning("Timeout sending reboot to Pico")
        # 2) chiudo la porta e aspetto il riavvio USB
        pico.close()
        time.sleep(3)  # attendi che il Pico riavvii e riattivi USB
        
        # 3) riapri la porta seriale
        pico = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1, write_timeout=0.1)
        time.sleep(2)   # attendi startup del firmware
        pico.reset_input_buffer()
        
        # 4) leggi fino al messaggio di ready
        start = time.time()
        while time.time() - start < 5:
            line = pico.readline().decode().strip()
            if "PICO IDLE" in line or "PICO READY" in line:
                app.logger.info("Pico is ready again")
                break
        else:
            app.logger.warning("Pico did not respond after reboot")
    
    threading.Thread(target=_do_reboot).start()
    return jsonify(status='rebooting pico')

# ---- Endpoints ----

@app.route('/')
def control():
    return render_template('control.html')

@app.route('/api/set_speed', methods=['POST'])
def set_speed():
    global current_speed
    if emergency_active:
        return jsonify(status='blocked'), 403

    # Only update the Pi's speed variable—do NOT send anything to the Pico
    current_speed = request.json.get('speed_pct', 100)
    return jsonify(status='speed set')


@app.route('/api/homing', methods=['POST'])
def homing():
    global emergency_active
    if emergency_active:
        return jsonify(status='blocked'), 403

    # Inoltra il comando homing in modo “fire-and-forget”
    try_write({"cmd": "homing"})
    # Torna sempre 200, il Pico gestirà il comando appena possibile
    return jsonify(status='homing sent')

@app.route('/api/jog', methods=['POST'])
def jog():
    global current_steps
    if emergency_active:
        return jsonify(status='blocked'), 403
    data = request.json
    axis_map = {'X':'BASE', 'Y':'M1', 'Z':'M2'}
    driver = axis_map.get(data['axis'])
    steps = int(data['mm'] * 80)
    cmd = {"axis": driver, "steps": steps, "speed_pct": current_speed}
    try_write(cmd)
    # Update state immediately
    idx = {'BASE':0,'M1':1,'M2':2}[driver]
    current_steps[idx] += steps
    return jsonify(status='jogged')

@app.route('/api/move_point', methods=['POST'])
def move_point():
    global current_steps
    if emergency_active:
        return jsonify(status='blocked'), 403
    x,y,z = request.json['x'], request.json['y'], request.json['z']
    gamma = math.degrees(math.atan2(y, x))
    r = math.hypot(x, y)
    x_p, z_p = r - dx, z - dz
    th1, th2 = inverse_kinematics(x_p, z_p)
    base_s = angle_to_steps(gamma)
    m1_s   = angle_to_steps(th1)
    m2_s   = angle_to_steps(th2)
    path   = interpolate(current_steps, [base_s, m1_s, m2_s])
    send_path(path)
    return jsonify(status='moved')

@app.route('/api/upload', methods=['POST'])
def upload():
    global command_queue
    if emergency_active:
        return jsonify(status='blocked'), 403
    command_queue = request.json.get('commands', [])
    return jsonify(status='queued', count=len(command_queue))

@app.route('/api/run_queue', methods=['POST'])
def run_queue():
    if emergency_active:
        return jsonify(status='blocked'), 403
    for cmd in command_queue:
        if emergency_active:
            break
        try_write(cmd)
    return jsonify(status='done')

@app.route('/api/stop', methods=['POST'])
def api_stop():
    global emergency_active
    emergency_active = True
    try_write({"cmd":"stop"})
    # Clear any pending queue
    return jsonify(status='stopped')

@app.route('/api/reset', methods=['POST'])
def api_reset():
    global emergency_active
    # 1) riattiva tutti i comandi lato Pi5
    emergency_active = False
    # 2) inoltra il reset allarmi al Pico
    try:
        pico.reset_output_buffer()
        pico.write((json.dumps({"cmd":"reset"}) + '\n').encode())
    except serial.SerialTimeoutException:
        app.logger.warning("Timeout sending reset to Pico")
    return jsonify(status='reset done')

if __name__=='__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
