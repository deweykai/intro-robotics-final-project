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
from robot import lidar, camera
import numpy as np
import copy
import logging

logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)

LIDAR_ANGLE_BINS = 667
# The sensor is so low it often hits the ground
LIDAR_SENSOR_MAX_RANGE = 2  # Meter
LIDAR_ANGLE_RANGE = math.radians(240)

lidar_offsets = np.linspace(
    +LIDAR_ANGLE_RANGE / 2.0, -LIDAR_ANGLE_RANGE / 2.0, LIDAR_ANGLE_BINS
)
# Only keep lidar readings not blocked by robot chassis
lidar_offsets = lidar_offsets[83: len(lidar_offsets) - 83]


def get_lidar_readings():
    readings = []
    pose_x, pose_y, pose_theta = loc.pose_x, loc.pose_y, loc.pose_theta
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83: len(
        lidar_sensor_readings) - 83]

    for alpha, rho in zip(lidar_offsets, lidar_sensor_readings):
        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha) * rho + 0.202
        ry = math.sin(alpha) * rho - 0.004

        # Convert detection from robot coordinates into world coordinates
        wx = math.cos(pose_theta) * rx - math.sin(pose_theta) * ry + pose_x
        wy = math.sin(pose_theta) * rx + math.cos(pose_theta) * ry + pose_y

        readings.append([wx, wy])

    return readings


color_ranges = []


def add_color_range_to_detect(lower_bound, upper_bound):
    """
    @param lower_bound: Tuple of BGR values
    @param upper_bound: Tuple of BGR values
    """
    global color_ranges
    logger.debug(f'Add [{lower_bound}, {upper_bound}] to detect')
    color_ranges.append(np.array([lower_bound, upper_bound]))


def check_if_color_in_range(bgr_tuple):
    """
    @param bgr_tuple: Tuple of BGR values
    @returns Boolean: True if bgr_tuple is in any of the color ranges specified in color_ranges
    """
    bgr_tuple = bgr_tuple[:3]
    for entry in color_ranges:
        lower, upper = entry[0], entry[1]
        if (bgr_tuple >= lower).all() and (bgr_tuple <= upper).all():
            return True
    return False


def detect_objects():
    objects = camera.getRecognitionObjects()
    rec = [(o.getPosition(), o.getColors()) for o in objects]
    return rec


def filter_colors(objects):
    filtered = list(filter(lambda c: check_if_color_in_range(c[1]), objects))
    return filtered


def detect_filtered_objects():
    return filter_colors(detect_objects())


def init():
    """Initialize vision module"""
    add_color_range_to_detect([0.9, 0.9, 0], [1.0, 1.0, 0.1])


def update():
    """Update hook for vision module"""
    rec = filter_colors(detect_objects())
    if len(rec) > 0:
        logger.debug(rec)
