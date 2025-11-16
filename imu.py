import sys
import os
import time
import numpy as np
"""
import scipy as scp
import matplotlib.pyplot as plt
"""
# access to the drivers
sys.path.append("../drivers-ddboat-v2")
import arduino_driver_v2 as arddrv
import imu9_driver_v2 as imudrv



def autocalibration(data_imu_mag):
    d =data_imu_mag
    M = np.vstack([d[:, 0]**2,
                   d[:, 1]**2,
                   d[:, 2]**2,
                   d[:, 0]*d[:, 1],
                   d[:, 0]*d[:, 2],
                   d[:, 1]*d[:, 2],
                   d[:, 0],
                   d[:, 1],
                   d[:, 2]]).T
    r = np.ones((1, 9))
    p = np.linalg.inv(M.T@M)@M.T # ce sont les paramètres de l'ellipsoide
    Q = 1/2*np.array([
        [2*p[0, 0], p[3, 0], p[4, 0]],
        [p[3, 0], 2*p[1, 0], p[5, 0]],
        [p[4, 0], p[5, 0], 2*p[2, 0]]
    ])
    b = -1/2*Q@p[7:].T
    y = scp.linalg.sqrtm(Q)@(data_imu_mag[:, :, None] - b)
    print(np.norm(y, axis = -1))
    return y


if __name__ == "__main__":
    imu = imudrv.Imu9IO()
    nb_points = 300
    data_imu_mag = np.zeros((nb_points, 3))
    
    for i in range(nb_points):
        data_imu_mag[i] = imu.read_mag_raw()
        print(imu.read_mag_raw(),imu.read_accel_raw(),imu.read_gyro_raw())
        time.sleep(0.1)
    
    np.save("log/data_imu_mag.npy", data_imu_mag)
    
    # with open("./data_imu_mag.txt", "w") as f:
    #     f.write(str(data_imu_mag))


    """
    # --- autocalibration ---
    y = autocalibration(data_imu_mag)

    # --- affichage ---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(data_imu_mag[:, 0], data_imu_mag[:, 1], data_imu_mag[:, 2],
               color='red', alpha=0.3, label='Avant calibration')
    ax.scatter(y[:, 0], y[:, 1], y[:, 2],
               color='green', alpha=0.6, label='Après calibration')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title('Calibration magnétomètre (avant/après)')

    plt.show()
    """

"""
ard = arddrv.ArduinoIO() # create an ARduino object
left_speed = 100
right_speed = -left_speed


ard.send_arduino_cmd_motor(left_speed,right_speed) # in place turn
time.sleep(2)
ard.send_arduino_cmd_motor(0,0) # stop
"""