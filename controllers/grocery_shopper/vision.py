"""Vision Processing

This module take vision sensor data and processes into a more usable format

This file provides:
    * lidar_hits - list of (x,y) coordinates where lidar

This module depends on:
    * localization - for caluclating the position of lidar_hits
"""


import numpy as np
import localization as loc
import math
from robot import lidar

LIDAR_ANGLE_BINS = 667
# The sensor is so low it often hits the ground
LIDAR_SENSOR_MAX_RANGE = 2  # Meter
LIDAR_ANGLE_RANGE = math.radians(240)


lidar_offsets = np.linspace(+LIDAR_ANGLE_RANGE /
                            2., -LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
# Only keep lidar readings not blocked by robot chassis
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83]


def get_lidar_readings():
    readings = []
    pose_x, pose_y, pose_theta = loc.pose_x, loc.pose_y, loc.pose_theta
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(
        lidar_sensor_readings)-83]

    for alpha, rho in zip(lidar_offsets, lidar_sensor_readings):
        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho + 0.202
        ry = math.sin(alpha)*rho - 0.004

        # Convert detection from robot coordinates into world coordinates
        wx = math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy = math.sin(pose_theta)*rx + math.cos(pose_theta)*ry + pose_y

        readings.append([wx, wy])

    return readings


def init():
    """Initialize vision module"""

    pass


def update():
    """Update hook for vision module"""

    # TODO: process lider readings into lider_hits
    pass
