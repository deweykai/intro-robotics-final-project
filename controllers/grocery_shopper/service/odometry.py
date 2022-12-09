"""Odometry

This is completely unusable. Most likely due to the issues the robot
has with turning.
"""

import matplotlib.pyplot as plt
import numpy as np
import robot
import bus


DT = robot.timestep / 1000


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * np.cos(x[2, 0]), 0],
                  [DT * np.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    x[2, 0] = pi_2_pi(x[2, 0])
    return x


def pi_2_pi(angle):
    return (angle + np.pi) % np.pi - np.pi


def calculate_u(vL, vR):
    vL_ms = vL / robot.MAX_SPEED * robot.MAX_SPEED_MS
    vR_ms = vR / robot.MAX_SPEED * robot.MAX_SPEED_MS
    v = (vL_ms + vR_ms) / 2
    yaw_rate = - vL_ms / (robot.AXLE_LENGTH / 2) + \
        vR_ms / (robot.AXLE_LENGTH / 2)

    u = np.array([v, yaw_rate]).reshape((2, 1))  # [v, yaw]
    return u


vL, vR = 0, 0

xEst = np.array([-5, 0, 0]).reshape((3, 1))


@bus.subscribe('/bot/wheel/cmd_vel/left', float)
def cmd_vel_left(value):
    global vL
    vL = value


@bus.subscribe('/bot/wheel/cmd_vel/right', float)
def cmd_vel_right(value):
    global vR
    vR = value


@bus.subscribe('/bot/cmd_tick', int)
def receive_landmarks(_):
    global xEst, PEst
    u = calculate_u(vL, vR)  # [v, d_yaw]
    xEst = motion_model(xEst, u)
    print(xEst)
