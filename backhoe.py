"""
Python:
- Forward Kinematics
- Inverse Kinematics
- Velocity Control Loop (constant)
- Position Control Loop (proportional)
- Serial communication
- 2D peak finding?

Arduino:
- Sensor reading
- Motor control
- Serial communication

Example:
a_measured: (65.01158903746261, -97.38577458793476)
a_expected: (67.5, -98)
a_error: 3.7%, 0.92% 
p_measured: (20.784, 17.314)
p_expected: (21.59, 16.51)
p_error: 3.75%, 4.85%
"""

from scipy.optimize import minimize, Bounds
import numpy as np
import math

def deg2rad(deg):
    return math.pi/180.0 * deg

def rad2deg(rad):
    return 180.0/math.pi * rad

def fkine(theta_boom, theta_stick):
    x_boom = 0.0
    z_boom = 0.0
    s_x = 25.4
    b_x = 12.3825

    x_stick = x_boom + s_x * math.cos(theta_boom)
    z_stick = z_boom + s_x * math.sin(theta_boom)

    x_bucket = x_stick + b_x * math.cos(theta_boom + theta_stick)
    z_bucket = z_stick + b_x * math.sin(theta_boom + theta_stick)

    return (x_bucket, z_bucket)


def ikine(theta_boom, theta_stick, x, z):
    def mini(v):
        pose = fkine(v[0], v[1])
        return math.sqrt((pose[0] - x)**2 + (pose[1] - z)**2)

    bounds = Bounds([deg2rad(-20), deg2rad(-120)],
                    [deg2rad(68), deg2rad(-60)])
    init = np.array([theta_boom, theta_stick])
    res = minimize(mini, init, bounds=bounds)
                  # options={'xtol': 1e-8, 'disp': True})
    return res.x


if __name__ == "__main__":
    #print fkine(deg2rad(-45), deg2rad(-45))
    x = 21.59
    z = 16.51
    theta_boom, theta_stick = ikine(deg2rad(0), deg2rad(0), x, z)
    print (rad2deg(theta_boom), rad2deg(theta_stick))
