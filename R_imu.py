import numpy as np
import time
import sys


np.set_printoptions(
    formatter={'float': '{:0.2f}'.format}
)

I = 64/180*np.pi

Z = np.array([[np.cos(I), 0, -np.sin(I)],
              [0, -np.cos(I), -np.cos(I)],
              [-np.sin(I), -np.sin(I), 0]])

Y = np.load("log/nord_ouest_haut_mercredisoir.npy").T

R_imu = Z@np.linalg.inv(Y)

print("Y : ", Y)
print("\n")
print("R_imu : ")
print(R_imu)
print("\n")
print(R_imu@Y[:, 0])
print(R_imu@Y[:, 1])
print(R_imu@Y[:, 2])



np.save("log/R_imu.npy", R_imu)