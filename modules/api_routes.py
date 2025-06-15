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
    # Utilizza lo stato corrente invece di valori di default
    with state.lock:
        return render_template('control.html',
            x = state.x,
            y = state.y,
            z = state.z,
            angle_j1 = state.angle_j1,
            angle_j2 = state.angle_j2,
            angle_j3 = state.angle_j3
        )

@bp.route('/video')
def video_page():
    return render_template('video.html')
    
@bp.route('/video_feed')
def video_feed():
    def generate():
        while True:
            # Usa un lock per accedere allo stato in modo sicuro
            with state.lock:
                frame = state.latest_frame
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)  # ~30 FPS per ridurre il carico
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@bp.route('/manual_gcode')
def manual_gcode():
    return render_template('manual_gcode.html')

@bp.route('/api/detections')
def api_detections():
    # Utilizza un timeout per evitare deadlock
    acquired = state.lock.acquire(timeout=0.1)
    if acquired:
        try:
            return jsonify({
                'frame_url': '/api/frame', 
                'detections': state.detections
            })
        finally:
            state.lock.release()
    else:
        return jsonify({
            'status': 'timeout',
            'message': 'State lock timeout'
        }), 503

@bp.route('/api/frame')
def frame():
    # Accesso sicuro allo stato con lock
    with state.lock:
        frame = state.latest_frame
    if not frame:
        return Response(status=204)
    return Response(frame, mimetype='image/jpeg')

@bp.route('/api/pico_status', methods=['GET'])
def pico_status():
    # Controlla se la connessione seriale è aperta
    ok = state.pico is not None and state.pico.is_open
    return jsonify(connected=ok)

@bp.route('/api/disconnect_pico', methods=['POST'])
def disconnect_pico():
    if state.pico:
        try:
            state.pico.close()
        except:
            pass
        state.pico = None
    return jsonify(status='disconnected')

@bp.route('/api/reconnect_pico', methods=['POST'])
def reconnect_pico():
    # Esegui la riconnessione in un thread separato
    threading.Thread(target=serial_comms.open_pico).start()
    return jsonify(status='reconnecting')

@bp.route('/api/set_speed', methods=['POST'])
def set_speed():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
    
    v = int(request.json.get('speed_pct', 100))
    state.current_speed = v
    lcd.set_speed(v)
    
    # Invia il comando senza attendere risposta
    serial_comms.try_write({"cmd": "speed", "speed_pct": v})
    return jsonify(status='ok')

@bp.route('/api/homing', methods=['POST'])
def homing():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
    
    try:
        target_angles = state.home_angles
        tolerance = 1.0
        
        for i, axis in enumerate(['BASE', 'M1', 'M2']):
            while True:
                with state.lock:
                    current_angle = [state.angle_j1, state.angle_j2, state.angle_j3][i]
                
                diff = target_angles[i] - current_angle
                if abs(diff) < tolerance:
                    break
                    
                direction = 1 if diff > 0 else -1
                serial_comms.try_write({
                    'axis': axis,
                    'mm': 0.1 * direction,
                    'speed_pct': 30
                })
                time.sleep(0.1)
        
        # Open gripper after homing
        serial_comms.try_write({'cmd': 'grip', 'angle': 0})
        state.homing_done = True
        return jsonify(status='homed')
    
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@bp.route('/api/interrupt_sequence', methods=['POST'])
def interrupt_sequence():
    state.sequence_interrupt = True
    return jsonify(status='interrupted')

@bp.route('/api/system_status', methods=['GET'])
def system_status():
    status = {
        'pico_connected': state.pico is not None and state.pico.is_open,
        'emergency_active': state.emergency_active,
        'command_queue_length': len(state.command_queue),
        'last_update': state.last_update,
        'fps': state.fps,
        'homing_done': state.homing_done,
        'sequence_running': state.sequence_running
    }
    return jsonify(status)

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
        # Imposta il flag per avviare la sequenza di visione
        state.start_sequence = True
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
    return jsonify(status='uploaded', count=len(cmds))

@bp.route('/api/run_queue', methods=['POST'])
def run_queue():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
    
    # Invia solo un flag, la coda verrà gestita dal thread
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
        
        # Riavvia l'app solo se ci sono aggiornamenti
        if result.returncode == 0 and "Already up to date" not in output:
            threading.Thread(target=lambda: (time.sleep(1), os._exit(0))).start()
        return jsonify(status=status, output=output)
    except Exception as e:
        return jsonify(status='error', output=str(e)), 500

@bp.route('/api/gcode', methods=['POST'])
def send_gcode():
    if state.emergency_active:
        return jsonify(status='blocked'), 403
        
    payload = request.json or {}
    code = payload.get('code', '')
    
    try:
        moves = motion_control.parse_gcode_to_moves(code)
        # Aggiungi i movimenti alla coda di comandi
        state.command_queue.extend(moves)
        return jsonify(status='ok', queued=len(moves))
    except Exception as e:
        return jsonify(status='error', error=str(e)), 500

@bp.route('/api/get_current_state', methods=['GET'])
def get_current_state():
    # Timeout per evitare deadlock
    acquired = state.lock.acquire(timeout=0.1)
    if acquired:
        try:
            return jsonify({
                'status': 'ok',
                'position': {
                    'x': state.x,
                    'y': state.y,
                    'z': state.z
                },
                'angles': {
                    'j1': state.angle_j1,
                    'j2': state.angle_j2,
                    'j3': state.angle_j3
                }
            })
        finally:
            state.lock.release()
    else:
        # Risposta di fallback in caso di timeout
        return jsonify({
            'status': 'timeout',
            'message': 'State lock timeout'
        }), 503


@bp.route('/api/clear_errors', methods=['POST'])
def clear_errors():
    # Reset degli errori senza influenzare lo stato operativo
    if state.pico and not state.pico.is_open:
        state.pico = None
    return jsonify(status='errors_cleared')