# -*- coding: utf-8 -*-
import time
import math
import numpy as np
import sys, os
from euler_angle import read_psi
from gps import prise_gps, cvt_gll_ddmm_2_dd
from pyproj import Proj
from control_utils import compute_motor_speeds

"""
Commentaires faits par ChatGPT et vérifiés par Clément SARTRE et Simon Jean KLECHA
"""



projDegree2Meter = Proj("+proj=utm +zone=30, +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs")




# --- Points fixes utilisés par le scénario aller/retour ---
reference_lat_ponton = 48.199020000000004
reference_lon_ponton = -3.0146599999999997

reference_lat_bouee = 48.199245
reference_lon_bouee = -3.0163166666

reference_x_ponton,reference_y_ponton = projDegree2Meter(reference_lon_ponton,reference_lat_ponton)
reference_x_bouee, reference_y_bouee = projDegree2Meter(reference_lon_bouee,reference_lat_bouee)


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
objectif_bouees = []
objectif_retours = []



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

    psi_d = 90
    Kp = 1.0      # à ajuster selon la sensibilité
    base_speed = 200  # vitesse moyenne de déplacement
    f = open("log.txt", "w")

    objectif_bouee = False
    objectif_retour = False

    t1 = time.time()

    t2 = time.time()

    # Lecture GPS initiale pour disposer d'une position avant la boucle principale
    while (t2 - t1) < 2:
        t2 = time.time()
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            lat,lon = cvt_gll_ddmm_2_dd(gll_data) # convert DDMM.MMMM to DD.DDDDD
            print(lat, lon)
            x,y = projDegree2Meter(lon, lat) # convert to meters


    cnt = 0

    while True:
        cnt += 1
        print("tour ", cnt)
        # --- Acquisition des mesures temps réel ---
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            lat,lon = cvt_gll_ddmm_2_dd(gll_data) # convert DDMM.MMMM to DD.DDDDD
            x,y = projDegree2Meter(lon, lat) # convert to meters
        t2 = time.time()
        if not objectif_bouee:
            # Navigation vers la bouée
            dx = reference_x_bouee - x
            dy = reference_y_bouee - y
            psi_d = np.arctan2(-dx, dy)*180/np.pi
            distance = np.sqrt(dx**2 + dy**2)
            if distance < 15:
                objectif_bouee = True
            #print(dx, dy, psi_d)
        elif not objectif_retour:
            # Puis navigation de retour au ponton
            dx = reference_x_ponton - x
            dy = reference_y_ponton - y
            psi_d = np.arctan2(-dx, dy)*180/np.pi
            distance = np.sqrt(dx**2 + dy**2)
            if distance < 10:
                objectif_retour = True
        else:
            # Mission terminée : on coupe les moteurs et on sauvegarde les logs
            ard.send_arduino_cmd_motor(0, 0)
            np.savez(
                "log/log_data_bouee.npz",
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
                objectif_bouee=np.array(objectif_bouees),
                objectif_retour=np.array(objectif_retours)
            )
            print("Données de fin sauvegardées dans log_data_bouee.npz")
            break


        


        
        psi = read_psi(imu)

        print(psi)

        print(psi_d)
        
        # Commande proportionnelle simple (heading) encapsulée dans control_utils
        left_speed, right_speed, erreur = compute_motor_speeds(psi, psi_d, base_speed, Kp)

        timestamp = t2 - t1

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
        objectif_bouees.append(objectif_bouee)
        objectif_retours.append(objectif_retour)


        f.write(str(t2 - t1) + str(" : ") + str(psi) + "\n, " + str(left_speed) + str(" ") + str(right_speed))
        # Envoi des vitesses (converties en entiers)
        ard.send_arduino_cmd_motor(int(left_speed), int(right_speed))
