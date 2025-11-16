"""
Utilities for turning raw magnetometer measurements into Euler angles for heading control.
Inclut la calibration du magnétomètre et la lecture directe du cap via l'IMU.
"""

import numpy as np
import time
import sys

sys.path.append("../drivers-ddboat-v2")
import arduino_driver_v2 as arddrv
import imu9_driver_v2 as imudrv


R_IMU = np.load("log/R_imu.npy")

def tolist(w):
    """Retourne w sous forme de liste simple (utile pour les vecteurs colonne numpy)."""
    if isinstance(w,(list)): return w
    return list(w[:, 0])

def adjoint(w):
    """Construit la matrice adjointe associée à w (2D ou 3D)."""
    if isinstance(w, (float, int)): return np.array([[0,-w] , [w,0]])
    #print('w=',w)
    w=tolist(w)
    #print('tolist(w)=',w)
    return np.array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])


def rotation_matrix(ycorr, a = np.array([[0, 0, 1]]).T):
    """Construit une matrice de rotation dont l'axe principal est le vecteur corrigé."""
    ycorr=ycorr.reshape(3, 1)
    C1 = ycorr - (ycorr.T@a)*a
    C1 = C1/np.linalg.norm(C1, ord = 2)
    C2 = adjoint(a)@C1
    return np.hstack([C1, C2, a]).T


def eulermat2angles(R):
    """Convertit une matrice de rotation en angles d'Euler (phi, theta, psi)."""
    #print("R : ", R)
    phi=np.arctan2(R[2,1],R[1,2])
    theta=-np.arcsin(R[2,0])
    psi=np.arctan2(R[1,0],R[0,0])
    return phi, theta, psi

def euler_angle(y, a = np.array([[0, 0, 1]]).T):
    """Applique la calibration magnétomètre puis retourne (y_corrigé, angles Euler)."""
    #print(np.shape(y))
    data = np.load("log/mag_calib.npz")
    bias = data["bias"] #pour recentrer
    #print(np.shape(bias))
    C = data["C"] #matrice de mise en sphère

####### Suite à une erreur de formule, nous avions mal implémenté notre code de sphérisation
####### et en nous servant d'un outil, se nommant ChatGPT, le code spherisation.py ne nous était
####### compréhensible et avons demandé l'aide à un professeur pour avoir des matrices qui 
####### fonctionnait de manière certaines, notre incertitude pouvaient en effet aussi provenir
####### de la matrice de rotation R_IMU pour alligner la base de l'IMU
####### avec le repère de référence choisi

    C = np.array([[ 2.90972176e-04,  3.43977509e-07, -2.05916088e-06],
                    [ 3.43977509e-07,  2.91312732e-04, -1.15352105e-06],
                    [-2.05916088e-06, -1.15352105e-06,  3.07914695e-04]]

    )
    bias = np.array([[ 2657.50199037],
                    [-3162.40097314],
                    [ 5738.29828085]])

    y = y - bias
    y = C@y
    y = y/np.linalg.norm(y, ord = 2)


    y = R_IMU@y # correction IMU

    return y, eulermat2angles(rotation_matrix(y))


def read_psi(imu):
    """Renvoie uniquement l'angle psi (cap) en degrés depuis l'IMU fournie."""
    y = np.array(imu.read_mag_raw())
    return euler_angle(y.reshape(3, 1))[1][2]*180/np.pi



if __name__ == "__main__":
    imu = imudrv.Imu9IO()

    nb_points = 3
    ycorrl = np.zeros((nb_points, 3))
    #y = np.load("log/raw_data1.npy")
    y = np.zeros((nb_points, 3))
    for i in range(nb_points):
        a = input("appuyer sur entrée pour acquerir la donnée" + str(i) + " : ")
        y[i] = np.array(imu.read_mag_raw())
        ycorr, (phi, theta, psi) = euler_angle(y[i].reshape(3, 1))
        print("phi : ", phi*180/np.pi, "theta : ", theta*180/np.pi, "psi : ", psi*180/np.pi)
        ycorrl[i] = ycorr.T

        time.sleep(0.1)

    np.save("log/ycorr.npy", ycorrl)
    np.save("log/y.npy", y)
