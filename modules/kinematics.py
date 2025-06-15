# modules/kinematics.py
import math

# Parametri cinematici (lunghezze dei bracci)
L1 = 120  # Lunghezza primo braccio (base al gomito) in mm
L2 = 100  # Lunghezza secondo braccio (gomito a polso) in mm

def forward_kinematics(angles):
    """
    Calcola la posizione cartesiana (x, y, z) per un robot articolato a 3 DOF
    Parametri di ingresso:
        angles: [theta1, theta2, theta3] in gradi
        theta1: angolo base (rotazione attorno all'asse Z)
        theta2: angolo primo giunto (gomito)
        theta3: angolo secondo giunto (polso)
    """
    # Converti angoli in radianti
    q1 = math.radians(angles[0])  # Base rotante
    q2 = math.radians(angles[1])  # Primo giunto
    q3 = math.radians(angles[2])  # Secondo giunto
    
    # Calcolo coordinate nel piano verticale XZ
    x_plane = L1 * math.cos(q2) + L2 * math.cos(q2 + q3)
    z_plane = L1 * math.sin(q2) + L2 * math.sin(q2 + q3)
    
    # Proiezione 3D con rotazione base
    x = x_plane * math.cos(q1)
    y = x_plane * math.sin(q1)
    z = z_plane
    
    return x, y, z