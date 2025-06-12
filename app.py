#!/usr/bin/env python3
import threading
from flask import Flask
from modules import serial_comms, vision, lcd, motion_control, api_routes

app = Flask(__name__, template_folder='templates', static_folder='static')

# Inizializza tutti i moduli
def initialize_modules():
    # Inizializza LCD
    lcd.initialize()
    lcd.set_status("SERVER START")
    
    # Apri connessione seriale con Pico
    serial_comms.open_pico()
    
    # Avvia thread di monitoraggio Pico
    threading.Thread(target=serial_comms.monitor_pico, daemon=True).start()
    
    # Avvia thread di gestione LCD
    threading.Thread(target=lcd.lcd_handler, daemon=True).start()
    
    # Avvia thread di visione
    threading.Thread(target=vision.capture_and_detect, daemon=True).start()
    
    # Avvia thread di gestione movimento
    threading.Thread(target=motion_control.motion_handler, daemon=True).start()

# Registra le route API
app.register_blueprint(api_routes.bp)

if __name__ == '__main__':
    initialize_modules()
    app.run(host='0.0.0.0', port=5000, threaded=True)