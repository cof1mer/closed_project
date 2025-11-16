"""
Fonctions utilitaires partagées par les contrôleurs (angles, saturation, vitesses).
"""


def normalize_angle(angle: float) -> float:
    """Contraint une différence d'angle à [-180, 180] degrés pour éviter les sauts."""
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    return angle


def clamp_speed(value: float, limit: int) -> float:
    """Sature une consigne moteur dans [-limit, limit]."""
    return max(min(value, limit), -limit)


def compute_motor_speeds(psi: float, psi_d: float, speed: float, Kp: float, limit: int = 255):
    """
    Retourne (left_speed, right_speed, erreur) pour un asservissement proportionnel simple.
    - psi: cap actuel mesuré
    - psi_d: cap désiré
    - speed: vitesse moyenne souhaitée (base_speed ou modulée)
    - Kp: gain proportionnel
    - limit: saturation absolue des moteurs
    """
    erreur = normalize_angle(psi_d - psi)
    correction = Kp * erreur
    left_speed = clamp_speed(speed - correction, limit)
    right_speed = clamp_speed(speed + correction, limit)
    return left_speed, right_speed, erreur
