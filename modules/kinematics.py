# modules/kinematics.py
import math

# Parametri cinematici (lunghezze dei bracci)
L1 = 140  # Lunghezza primo braccio (base al gomito) in mm
L2 = 140  # Lunghezza secondo braccio (gomito a polso) in mm

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

def inverse_kinematics(x, y, z):
    """
    Calcola gli angoli dei giunti per una posizione cartesiana specifica
    Restituisce: [theta1, theta2, theta3] in gradi
    """
    # Calcola angolo base (rotazione attorno a Z)
    theta1 = math.degrees(math.atan2(y, x))
    
    # Calcola distanza nel piano X-Y
    r = math.sqrt(x**2 + y**2)
    
    # Calcola distanza euclidea nel piano verticale
    d = math.sqrt(r**2 + z**2)
    
    # Legge dei coseni per triangoli
    alpha = math.acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))
    beta = math.acos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))
    
    # Calcola angoli dei giunti
    theta2 = math.degrees(math.atan2(z, r) + alpha)
    theta3 = math.degrees(beta - math.pi)  # -π per estensione completa
    
    return [theta1, theta2, theta3]