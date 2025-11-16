# -*- coding: utf-8 -*-
import time
import math
import numpy as np
import sys, os
from euler_angle2 import read_psi, read_psi_d, set_psi_d_deg


# Accès driver Arduino (adapté à ton arborescence)
sys.path.append("../drivers-ddboat-v2")
import arduino_driver_v2 as arddrv

# =========================
#   Outils angle & clamp
# =========================
def wrap_pi(a):
    """Enroule un angle en radians dans [-pi, pi)."""
    a = (a + math.pi) % (2*math.pi) - math.pi
    return a

def ang_diff(a, b):
    """Retourne (a - b) replié dans [-pi, pi). Utile pour erreurs angulaires."""
    return wrap_pi(a - b)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# =========================
#   PID cap (avec anti-windup et D sur mesure)
# =========================
class HeadingPID:
    def __init__(self, Kp=2.0, Ki=0.5, Kd=0.2, I_max=0.6, u_max=1.0):
        """
        Kp, Ki, Kd : gains du PID (sur l'erreur angulaire en rad).
        I_max : borne absolue sur l'intégrale (rad) pour anti-windup.
        u_max : sortie PID bornée dans [-u_max, u_max] (u est sans unité, avant mise à l'échelle moteurs).
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.I_max = abs(I_max)
        self.u_max = abs(u_max)
        self.e_int = 0.0
        self.prev_psi = None  # pour D sur la mesure (évite "derivative kick")

    def reset(self, psi_now=None):
        self.e_int = 0.0
        self.prev_psi = psi_now

    def update(self, psi, psi_d, dt):
        """
        psi  : cap mesuré (rad)
        psi_d: cap désiré (rad)
        dt   : pas de temps (s)
        Retourne u in [-u_max, u_max]
        """
        # Erreur angulaire courte (psi_d - psi) repliée
        e = ang_diff(psi_d, psi)

        # Intégrale avec clamp
        self.e_int += e * dt
        self.e_int = clamp(self.e_int, -self.I_max, self.I_max)

        # Dérivée sur la mesure (plus robuste au pas de consigne)
        dpsi = 0.0
        if self.prev_psi is not None and dt > 1e-6:
            dpsi = ang_diff(psi, self.prev_psi) / dt
        self.prev_psi = psi

        # PID : u = Kp*e + Ki*∫e - Kd*dpsi
        u = self.Kp*e + self.Ki*self.e_int - self.Kd*dpsi

        # Saturation sortie + anti-windup conditionnel (stopper l'intégration si saturé et e pousse dans le mauvais sens)
        if abs(u) > self.u_max:
            u_sat = clamp(u, -self.u_max, self.u_max)
            # Anti-windup simple: si saturation et u et e de même signe, on annule la dernière intégration
            if (u * e) > 0:
                self.e_int -= e * dt
            u = u_sat

        return u, e

# =========================
#   Mélange moteurs
# =========================
def mix_differential(u, throttle=0.0, max_cmd=200):
    """
    u        : commande lacet normalisée dans [-1, 1] (sortie PID / u_max).
    throttle : avance (normalisée [-1, 1]) ; 0.0 = rotation pure, >0 avance.
    max_cmd  : amplitude max envoyée aux moteurs (ex: 200 sur 255).
    Retour: (left_cmd, right_cmd) en entiers.
    """
    u = clamp(u, -1.0, 1.0)
    throttle = clamp(throttle, -1.0, 1.0)
    # Convention: u>0 => tourner à droite (tribord) => on pousse plus à gauche
    left  = throttle + u
    right = throttle - u
    # Rebornage et échelle en PWM
    left  = clamp(left,  -1.0, 1.0) * max_cmd
    right = clamp(right, -1.0, 1.0) * max_cmd
    return int(round(left)), int(round(right))

# =========================
#   Boucle principale
# =========================
def control_heading_loop(read_psi, read_psi_d, base_throttle=0.0,
                         loop_hz=20.0, max_cmd=200,
                         Kp=2.0, Ki=0.5, Kd=0.2):
    """
    read_psi()  -> retourne le cap mesuré (rad).
    read_psi_d()-> retourne la consigne cap (rad).
    base_throttle : avance de base ([-1,1]) ; pour tenir un cap en avançant.
    loop_hz       : fréquence de boucle (ex: 20 Hz).
    max_cmd       : amplitude max vers moteurs (ex: 200).
    Gains PID     : Kp, Ki, Kd.
    """
    ard = arddrv.ArduinoIO()
    pid = HeadingPID(Kp=Kp, Ki=Ki, Kd=Kd, I_max=0.8, u_max=1.0)

    dt_target = 1.0 / loop_hz
    t_prev = time.monotonic()
    pid.reset(psi_now=read_psi())

    try:
        while True:
            t_now = time.monotonic()
            dt = max(1e-3, t_now - t_prev)
            t_prev = t_now

            psi   = read_psi()
            psi_d = read_psi_d()

            u, e = pid.update(psi, psi_d, dt)  # u in [-1,1] (car u_max=1)
            # erreur angulaire repliée déjà calculée par le PID :
            e = ang_diff(psi_d, psi)

            # modulation de la propulsion selon l'alignement
            # erreur angulaire déjà calculée : e = psi_d - psi (repliée)
            e_abs = abs(e)

            # paramètres vitesse
            throttle_min = 0.05      # pousse minimale (moteurs toujours un peu actifs)
            throttle_max = base_throttle  # pousse max face au cap
            k = 2.0                  # plus grand = plus "pointu" (avance fort seulement si bien orienté)

            # modulation continue
            throttle = throttle_min + (throttle_max - throttle_min) * max(0, np.cos(k * e_abs))

            # (optionnel) couper avance s’il est complètement à l’envers
            if e_abs > np.pi/2:   # > 90°
                throttle = 0.0

            left_cmd, right_cmd = mix_differential(u, throttle=throttle, max_cmd=max_cmd)


            # Envoi commandes
            arddrv_ok = ard.send_arduino_cmd_motor(left_cmd, right_cmd)

            # (Optionnel) petit log console
            # print(f"psi={psi:.2f} rad  psi_d={psi_d:.2f}  err={e:.2f}  u={u:.2f}  L/R={left_cmd}/{right_cmd}")

            # asservissement à fréquence fixe
            t_end = time.monotonic()
            sleep_time = dt_target - (t_end - t_now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        # sécurité : stop
        ard.send_arduino_cmd_motor(0, 0)

# =========================
#   Exemple d’intégration
# =========================
# Remplace ces deux fonctions par tes vraies sources (fusion IMU pour psi, consigne pour psi_d).
def _example_read_psi():
    # TODO: retourner le cap mesuré en radians (e.g., depuis ta fusion IMU)
    # Ex: return current_heading_rad
    raise NotImplementedError

def _example_read_psi_d():
    # TODO: retourner la consigne cap en radians
    # Ex: une variable globale, ou un setpoint provenant d'un planificateur
    raise NotImplementedError

# Pour lancer :
# control_heading_loop(_example_read_psi, _example_read_psi_d, base_throttle=0.3, loop_hz=20, max_cmd=200, Kp=2.0, Ki=0.4, Kd=0.15)


if __name__ == "__main__":
    # Exemple : cap désiré à 45° au démarrage (modifiable à chaud via set_psi_d_deg())
    # set_psi_d_deg(360)

    # control_heading_loop(
    #     read_psi=read_psi,
    #     read_psi_d=read_psi_d,
    #     base_throttle=0.9,     # avance de base (à ajuster)
    #     loop_hz=20.0,
    #     max_cmd=200,
    #     Kp=2.0, Ki=0.4, Kd=0.15
    # )
    # test à vide ou sur plan d’eau calme
    ard = arddrv.ArduinoIO()
    ard.send_arduino_cmd_motor(255, 255)   # même vitesse sur les deux moteurs
    time.sleep(10)
    ard.send_arduino_cmd_motor(0, 0)

