
# launcher_and_console.py
"""
Script per avviare il server Flask e aprire una console HTTP verso l'interfaccia di app.py.
In questo modo solo app.py comunica con il Pico via seriale.
Uso:
 1. Installa dipendenze: pip install flask requests
 2. Posiziona questo script nella stessa cartella di app.py
 3. Esegui: python launcher_and_console.py
 4. Nel prompt digita comandi JSON o azioni (move, upload, run_queue, stop, reset)
    - move x y z
    - grip angle
    - upload [JSON array]
    - run_queue
    - stop
    - reset
    - exit
"""
import subprocess
import sys, os, time, json
import requests

# Configurazione
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FLASK_SCRIPT = os.path.join(BASE_DIR, 'app.py')
FLASK_CMD = [sys.executable, FLASK_SCRIPT]
API_URL = 'http://127.0.0.1:5000/api'

# Avvio server Flask
print(f"Avvio server Flask: {FLASK_CMD}")
server = subprocess.Popen(FLASK_CMD)
print(f"Server Flask avviato (PID {server.pid})")

time.sleep(2)  # attesa avvio

# Funzioni HTTP

def move(x, y, z):
    resp = requests.post(f"{API_URL}/move_point", json={'x': x, 'y': y, 'z': z})
    print(resp.json())

def upload(cmds):
    resp = requests.post(f"{API_URL}/upload", json={'commands': cmds})
    print(resp.json())

def run_queue():
    resp = requests.post(f"{API_URL}/run_queue")
    print(resp.json())

# Comandi emergenza

def stop():
    upload([{'cmd':'stop'}]); run_queue()

def reset():
    upload([{'cmd':'reset'}]); run_queue()

# Console interattiva
while True:
    try:
        line = input('> ').strip()
    except EOFError:
        break
    if not line:
        continue
    if line.lower() in ('exit','quit'):
        break
    parts = line.split()
    cmd = parts[0].lower()
    try:
        if cmd == 'move' and len(parts) == 4:
            move(float(parts[1]), float(parts[2]), float(parts[3]))
        elif cmd == 'grip' and len(parts) == 2:
            angle = float(parts[1])
            upload([{'cmd':'grip', 'angle':angle}]); run_queue()
        elif cmd == 'upload':
            # interpret rest as JSON
            cmds = json.loads(line[len('upload '):])
            upload(cmds)
        elif cmd == 'run_queue':
            run_queue()
        elif cmd == 'stop':
            stop()
        elif cmd == 'reset':
            reset()
        else:
            print('Comando non riconosciuto')
    except Exception as e:
        print('Errore:', e)

# Termina server Flask
print('Shutting down Flask server...')
server.terminate()
server.wait()
print('Server stopped.')
