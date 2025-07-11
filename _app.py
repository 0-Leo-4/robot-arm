#!/usr/bin/env python3
import threading
from flask import Flask
from modules import serial_comms, vision, lcd, motion_control, api_routes, encoders, kinematics
import sys
import os

app = Flask(__name__, template_folder='templates', static_folder='static')
sys.path.append(os.path.join(os.path.dirname(__file__), 'modules'))

# Inizializza tutti i moduli
def initialize_modules():
    # Prima apri la connessione seriale al Pico
    serial_comms.open_pico()
    
    # Poi inizializza LCD
    lcd.initialize()
    
    # Avvia thread di gestione LCD
    threading.Thread(target=lcd.lcd_handler, daemon=True).start()
    
    # Avvia thread di visione
    threading.Thread(target=vision.capture_and_detect, daemon=True).start()
    
    # Avvia thread di gestione movimento
    threading.Thread(target=motion_control.motion_handler, daemon=True).start()
    
    # Avvia thread lettura encoder
    threading.Thread(target=encoders.encoder_reader.run, daemon=True).start()

# Registra le route API
app.register_blueprint(api_routes.bp)

if __name__ == '__main__':
    initialize_modules()
    app.run(host='0.0.0.0', port=5000, threaded=True)