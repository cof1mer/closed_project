# -*- coding: utf-8 -*-
import time
import math
import numpy as np
import sys, os
from euler_angle import read_psi
from gps import prise_gps, cvt_gll_ddmm_2_dd
from pyproj import Proj
from control_utils import compute_motor_speeds



projDegree2Meter = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")




# --- Déclaration des points et paramètres de la trajectoire ---

reference_lat_ponton = 48.199020000000004
reference_lon_ponton = -3.0146599999999997

reference_lat_bouee = 48.199245
reference_lon_bouee = -3.0163166666

center_latlon = [48.199707, -3.018784]

reference_x_ponton,reference_y_ponton = projDegree2Meter(reference_lon_ponton,reference_lat_ponton)
reference_x_bouee, reference_y_bouee = projDegree2Meter(reference_lon_bouee,reference_lat_bouee)

bouee = np.array([reference_x_bouee, reference_y_bouee])
center = projDegree2Meter(center_latlon[1], center_latlon[0])
ponton = np.array([reference_x_ponton, reference_y_ponton])
R1 = 240 # en metres
T_c = 3000
w1 = -1/(3*R1)
t_off = 100
N = 5
moii = 2 # numéro qu'on m'attribue au sein du groupe

def R2(t):
    """Rayon interne (en m) de la formation qui décroît avec le temps."""
    return 10*np.exp(-t/200) + 5

def w2(t):
    """Vitesse angulaire de la cible mobile sur le petit cercle."""
    return 1/(2*R2(t))


def p(t):
    """Position du centre de la formation sur la grande orbite."""
    return center + R1*np.array([np.cos(w1*(t + t_off)), np.sin(w1*(t + t_off))])

def v(t, i = 3):
    """Déplacement relatif du i-ème bateau autour du centre."""
    w = w2(t)
    return R2(t)*np.array([np.cos(w*t + 2*np.pi*i/N), np.sin(w*t + 2*np.pi*i/N)])

def a(t):
    """Point cible absolu pour notre bateau (centre + offset)."""
    return p(t) + v(t, moii)

def Kpv(d):
    """Gain proportionnel sur la vitesse en fonction de la distance à la cible."""
    return 1 - (-np.arctan(d - 5)/np.arctan(5) + 1)/2

# --- Initialisation des listes pour stocker les logs ---
timestamps = []
lats = []
lons = []
xs = []
ys = []
dxs = []
dys = []
distances = []
psis = []
psis_d = []
erreurs = []
left_speeds = []
right_speeds = []
points_suivis = []


# Accès driver Arduino (adapté à ton arborescence)
sys.path.append("../drivers-ddboat-v2")
import arduino_driver_v2 as arddrv
import imu9_driver_v2 as imudrv
sys.path.append("..")
import gps_driver_v2 as gpddrv

if __name__ == "__main__":
    ard = arddrv.ArduinoIO()
    imu = imudrv.Imu9IO()
    gps = gpddrv.GpsIO()

    objectif_retour = False

    psi_d = 90
    Kp = 1.0      # à ajuster selon la sensibilité
    base_speed = 200  # vitesse moyenne de déplacement
    f = open("log.txt", "w")

    # Heure cible (en heures, minutes et secondes)
    target_hour = 10
    target_min = 55
    target_sec = 00

    # On calcule le timestamp cible du jour courant
    now = time.localtime()
    target_time = time.mktime((
        now.tm_year, now.tm_mon, now.tm_mday,
        target_hour, target_min, target_sec,
        0, 0, -1
    ))

    # Si l'heure est déjà passée, on vise demain
    if target_time <= time.time():
        target_time += 24 * 3600

    print("Attente jusqu’à", time.strftime("%H:%M:%S", time.localtime(target_time)))

# Boucle d’attente active (simple mais précise)
    tf = 750


    while True:
        # Attente active jusqu'au créneau demandé tout en gardant le GPS chaud
        remaining = target_time - time.time()
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            lat,lon = cvt_gll_ddmm_2_dd(gll_data) # convert DDMM.MMMM to DD.DDDDD
            print(lat, lon)
            x,y = projDegree2Meter(lon, lat) # convert to meters
        if remaining <= 0:
            break
        elif remaining > 1:
            # On dort longtemps au début
            time.sleep(remaining / 2)
        else:
            # On dort par petites fractions de seconde pour plus de précision
            time.sleep(0.0005)  # 0.5 millisecondes

    t1 = time.time()

    t2 = time.time()

    cnt = 0

    while True:
        cnt += 1
        print("tour ", cnt)
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            lat,lon = cvt_gll_ddmm_2_dd(gll_data) # convert DDMM.MMMM to DD.DDDDD
            x,y = projDegree2Meter(lon, lat) # convert to meters
        pos = np.array([x, y])
        t2 = time.time()

        timestamp = t2 - t1

        psi = read_psi(imu)

        if (t2 - t1) < tf:
            # --- Phase de poursuite de la trajectoire virtuelle ---
            dennemi = a(t2 -t1) - pos
            dx = dennemi[0]
            dy = dennemi[1]
            psi_d = np.arctan2(-dx, dy)*180/np.pi
            distance = np.sqrt(dx**2 + dy**2)

            Kv = Kpv(distance)
            speed = base_speed * Kv

            left_speed, right_speed, erreur = compute_motor_speeds(psi, psi_d, speed, Kp)

        
        else:
            # --- Phase de retour au ponton ---
            dx = reference_x_ponton - x
            dy = reference_y_ponton - y
            psi_d = np.arctan2(-dx, dy)*180/np.pi
            distance = np.sqrt(dx**2 + dy**2)

            if distance < 5:
            
                left_speed, right_speed, erreur = compute_motor_speeds(psi, psi_d, base_speed, Kp)

                timestamp = t2 - t1
            else:
                ard.send_arduino_cmd_motor(0, 0)
                print("Fini")
                break



        f.write(str(t2 - t1) + str(" : ") + str(psi) + "\n, " + str(left_speed) + str(" ") + str(right_speed))
        # Envoi des vitesses (converties en entiers)
        ard.send_arduino_cmd_motor(int(left_speed), int(right_speed))


        # On empile les données dans les listes
        timestamps.append(timestamp)
        lats.append(lat)
        lons.append(lon)
        xs.append(x)
        ys.append(y)
        dxs.append(dx)
        dys.append(dy)
        distances.append(distance)
        psis.append(psi)
        psis_d.append(psi_d)
        erreurs.append(erreur)
        left_speeds.append(left_speed)
        right_speeds.append(right_speed)
        points_suivis.append(a(t2 -t1))


        if not cnt % 100:
            # Sauvegarde périodique pour éviter de perdre les données en cas d'arrêt
            np.savez(
                    "log/log_data_suivi.npz",
                    time=np.array(timestamps),
                    lat=np.array(lats),
                    lon=np.array(lons),
                    x=np.array(xs),
                    y=np.array(ys),
                    dx=np.array(dxs),
                    dy=np.array(dys),
                    distance=np.array(distances),
                    psi=np.array(psis),
                    psi_d=np.array(psis_d),
                    erreur=np.array(erreurs),
                    left_speed=np.array(left_speeds),
                    right_speed=np.array(right_speeds),
                    points_suivis = np.array(points_suivis)
                )
            print("Données de fin sauvegardées dans log_data_suivi.npz")
