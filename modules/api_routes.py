# modules/api_routes.py
from flask import Blueprint, request, jsonify, render_template, Response
import subprocess
import time
import os
import json
import threading
import numpy as np
from . import serial_comms, lcd, motion_control
from .shared_state import state

# Blueprint per le route API
bp = Blueprint('api', __name__, template_folder='templates', static_folder='static')

# Variabile per il relè di alimentaione
RELAY_GPIO = 17

@bp.route('/')
def index():
    return render_template('control.html',
        x = state.x,
        y = state.y,
        z = state.z,
        j1 = state.angle_j1,
        j2 = state.angle_j2,
        j3 = state.angle_j3
    )

@bp.route('/video')
def video_page():
    return render_template('video.html')
    
@bp.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if state.latest_frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + state.latest_frame + b'\r\n')
            time.sleep(0.016)  # ~60 FPS
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@bp.route('/manual_gcode')
def manual_gcode():
    return render_template('manual_gcode.html')

@bp.route('/api/detections')
def api_detections():
    with state.lock:
        return jsonify({
            'frame_url': '/api/frame', 
            'detections': state.detections
        })

@bp.route('/api/frame')
def frame():
    if not state.latest_frame:
        return Response(status=204)
    return Response(state.latest_frame, mimetype='image/jpeg')

@bp.route('/api/pico_status', methods=['GET'])
def pico_status():
    if not state.pico or not state.pico.is_open:
        serial_comms.open_pico()
    ok = state.pico is not None and state.pico.is_open
    return jsonify(connected=ok)

@bp.route('/api/disconnect_pico', methods=['POST'])
def disconnect_pico():
    if state.pico:
        state.pico.close()
        state.pico = None
    return jsonify(status='disconnected')

@bp.route('/api/reconnect_pico', methods=['POST'])
def reconnect_pico():
    serial_comms.open_pico()
    return jsonify(status='reconnected' if state.pico else 'error')

@bp.route('/api/set_speed', methods=['POST'])
def set_speed():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
    v = int(request.json.get('speed_pct', 100))
    state.current_speed = v
    lcd.set_speed(v)
    serial_comms.try_write({"cmd": "speed", "speed_pct": v})
    return jsonify(status='ok')

@bp.route('/api/homing', methods=['POST'])
def homing():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
    serial_comms.try_write({"cmd": "homing"})
    return jsonify(status='ok')

@bp.route('/api/stop', methods=['POST'])
def stop():
    state.emergency_active = True
    serial_comms.try_write({"cmd": "stop"})
    state.relay.on()
    return jsonify(status='stopped')

@bp.route('/api/reset', methods=['POST'])
def reset_alarm():
    state.emergency_active = False
    serial_comms.try_write({"cmd": "reset"})
    state.relay.off()
    return jsonify(status='reset')

@bp.route('/api/start_sequence', methods=['POST'])
def start_sequence():
    try:
        
        return jsonify(status='started')
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@bp.route('/api/gpio', methods=['POST'])
def set_gpio():
    data = request.json
    pin = int(data.get('pin', RELAY_GPIO))
    state_val = int(data.get('state', 0))
    try:
        if pin == RELAY_GPIO:
            if state_val:
                state.relay.on()
            else:
                state.relay.off()
            return jsonify(status='ok', pin=pin, state=state_val)
        else:
            # Per altri pin, usa gpiozero dinamicamente
            from gpiozero import OutputDevice
            dev = OutputDevice(pin)
            if state_val:
                dev.on()
            else:
                dev.off()
            return jsonify(status='ok', pin=pin, state=state_val)
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@bp.route('/api/upload', methods=['POST'])
def upload_commands():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
    cmds = request.json.get('commands', [])
    state.command_queue = cmds
    return jsonify(status='uploaded')

@bp.route('/api/run_queue', methods=['POST'])
def run_queue():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
    # La coda verrà gestita dal thread motion_handler
    return jsonify(status='queue_started')

@bp.route('/api/reboot_pi', methods=['POST'])
def reboot_pi():
    try:
        threading.Thread(target=lambda: os.system('sudo reboot')).start()
        return jsonify(status='rebooting')
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@bp.route('/api/reboot_pico', methods=['POST'])
def reboot_pico():
    try:
        if state.pico:
            state.pico.setDTR(False)
            time.sleep(0.2)
            state.pico.setDTR(True)
            time.sleep(0.2)
        return jsonify(status='rebooted')
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@bp.route('/api/git_pull', methods=['POST'])
def git_pull():
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
            threading.Thread(target=lambda: (time.sleep(1), os._exit(0)).start())
        return jsonify(status=status, output=output)
    except Exception as e:
        return jsonify(status='error', output=str(e)), 500

@bp.route('/api/gcode', methods=['POST'])
def send_gcode():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
        
    payload = request.json or {}
    code = payload.get('code', '')
    moves = motion_control.parse_gcode_to_moves(code)
    
    # Aggiungi i movimenti alla coda di comandi
    state.command_queue.extend(moves)
    return jsonify(status='ok', queued=len(moves))

@bp.route('/api/get_current_state', methods=['GET'])
def get_current_state():
    # Manda due comandi al Pico: angoli e posizione
    serial_comms.try_write({"cmd": "getenc"})
    serial_comms.try_write({"cmd": "getpos"})
    # Attendi brevemente che il reader thread consumi e aggiorni lo state
    time.sleep(0.05)

    # Ora torna il JSON unificato
    with state.lock:
        return jsonify({
            'status': 'ok',
            'position': {
                'x': state.x,
                'y': state.y,
                'z': state.z
            }
        })
        
@bp.route('/api/get_current_angles', methods=['GET'])
def get_current_angles():
    # Se i dati sono vecchi, richiedi un aggiornamento
    if not state.angles_fresh:
        serial_comms.try_write({"cmd": "getenc"})
    
    with state.lock:
        return jsonify({
            'j1': state.angle_j1,
            'j2': state.angle_j2,
            'j3': state.angle_j3,
            'fresh': state.angles_fresh,
            'timestamp': state.last_angle_update
        })

# @bp.route('/api/move_to_object', methods=['POST'])
# def move_to_object():
#     """Muovi il robot verso un oggetto rilevato"""
#     if state.emergency_active:
#         return jsonify(status='blocked'), 403
    
#     data = request.json
#     obj_id = data.get('object_id')
    
#     # Trova l'oggetto nelle rilevazioni correnti
#     with state.lock:
#         obj = next((d for d in state.detections if d['id'] == obj_id), None)
    
#     if not obj:
#         return jsonify(status='error', error='Oggetto non trovato'), 404
    
#     try:
#         # Converti coordinate visione → robot
#         robot_x, robot_y = calibration.vision_to_robot_coordinates(obj['x'], obj['y'])
        
#         # Altezza di sicurezza (da configurare)
#         safe_z = 50.0  
        
#         # Altezza di pick (da configurare)
#         pick_z = 10.0   
        
#         # Sequenza di movimenti
#         move_sequence = [
#             {"axis": "BASE", "mm": robot_x, "speed_pct": 70},
#             {"axis": "M1", "mm": robot_y, "speed_pct": 70},
#             {"axis": "M2", "mm": safe_z, "speed_pct": 50},
#             {"axis": "M2", "mm": pick_z, "speed_pct": 30},
#             {"cmd": "grip", "angle": 80},  # Angolo chiusura
#             {"axis": "M2", "mm": safe_z, "speed_pct": 50}
#         ]
        
#         # Aggiungi alla coda di comandi
#         state.command_queue.extend(move_sequence)
#         return jsonify(status='pick_queued', moves=len(move_sequence))
    
#     except Exception as e:
#         return jsonify(status='error', error=str(e)), 500