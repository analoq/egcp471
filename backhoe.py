import math
import time

from scipy.optimize import minimize, Bounds
import numpy as np
import serial

COM_PATH = '/dev/cu.usbmodem1411'
GRAVITY = 9.807

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

def get_boom_angle():
    com_port.write([0x11, 0x00, 0x00])
    value = float(com_port.readline())
    ratio = value/GRAVITY
    if ratio > 1.0:
        ratio = 1.0
    elif ratio < -1.0:
        ratio = -1.0
    return -math.asin(ratio)

def get_stick_angle():
    boom_angle = get_boom_angle()
    com_port.write([0x12, 0x00, 0x00])
    value = float(com_port.readline())
    ratio = value/GRAVITY
    if ratio > 1.0:
        ratio = 1.0
    elif ratio < -1.0:
        ratio = -1.0
    abs_stick_angle = -math.asin(ratio)
    return abs_stick_angle - boom_angle

def fire(joint, delta_theta):
    DIR_POS = 0x01
    DIR_NEG = 0x00
    joints = {
        'boom': {'address': 0x01,
                 'magnitude': 0xA0,
                 'pos_secrad': 1.0 / deg2rad(31.654),
                 'neg_secrad': 1.0 / deg2rad(41.607)},
        'stick': {'address': 0x02,
                  'magnitude': 0xC0,
                  'pos_secrad': 1.0 / deg2rad(19.95),
                  'neg_secrad': 1.0 / deg2rad(22.22)},
        'bucket': {'address': 0x03,
                   'magnitude': 0xFF,
                   'pos_secrad': 1.0 / deg2rad(61.538),
                   'neg_secrad': 1.0 / deg2rad(88.495)},
        'house': {'address': 0x04,
                  'magnitude': 0xA0,
                  'pos_secrad': 1.0 / deg2rad(47.368),
                  'neg_secrad': 1.0 / deg2rad(47.368)},
    }

    if delta_theta > 0:
        com_port.write([joints[joint]['address'], DIR_POS, joints[joint]['magnitude']])
        time.sleep(joints[joint]['pos_secrad'] * delta_theta)
    else:
        com_port.write([joints[joint]['address'], DIR_NEG, joints[joint]['magnitude']])
        time.sleep(joints[joint]['neg_secrad'] * -delta_theta)
    com_port.write([joints[joint]['address'], DIR_NEG, 0x00])

def control(x, y):
    theta_boom = get_boom_angle()
    theta_stick = get_stick_angle()
    dest_theta_boom, dest_theta_stick = ikine(theta_boom, theta_stick, x, y)

    # move boom
    count = 4
    while True:
        delta_theta_boom = dest_theta_boom - theta_boom
        print rad2deg(delta_theta_boom)
        if abs(delta_theta_boom) < deg2rad(3):
            break
        print rad2deg(delta_theta_boom)
        fire('boom', delta_theta_boom)
        time.sleep(0.5)
        theta_boom = get_boom_angle()
        count -= 1
        if count == 0:
            break


    # move stick
    count = 4
    while True:
        delta_theta_stick = dest_theta_stick - theta_stick
        if abs(delta_theta_stick) < deg2rad(3):
            break
        print rad2deg(delta_theta_stick)
        fire('stick', delta_theta_stick)
        time.sleep(0.5)
        theta_stick = get_stick_angle()
        count -= 1
        if count == 0:
            break


def pickup():
    fire('bucket', deg2rad(-120))

def dropoff():
    fire('bucket', deg2rad(80))

def diagnostics():
    com_port.write([0x00, 0x00, 0x00])
    line = com_port.readline()
    if line.strip() == 'OK':
        print 'Diagnostic pass'
        return True
    else:
        print 'Diagnostic fail'
        return False

if __name__ == "__main__":
    com_port = serial.Serial(COM_PATH, timeout=60)
    time.sleep(1.0)
    diagnostics()
    control(10, 20)
    #control(30, 20)
    #dropoff()
    #time.sleep(1.0)
    #control(15, -1)
    #pickup()
    #fire('house', deg2rad(-30))
    com_port.close()
