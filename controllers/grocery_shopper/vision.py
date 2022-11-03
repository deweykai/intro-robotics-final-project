"""Vision Processing

This module take vision sensor data and processes into a more usable format

This file provides:
    * lidar_hits - list of (x,y) coordinates where lidar

This module depends on:
    * localization - for caluclating the position of lidar_hits
"""


import numpy as np
import math

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5  # Meters
LIDAR_ANGLE_RANGE = math.radians(240)


lidar_sensor_readings = []  # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE /
                            2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
# Only keep lidar readings not blocked by robot chassis
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83]


def init():
    """Initialize vision module"""

    pass


def update():
    """Update hook for vision module"""

    # TODO: process lider readings into lider_hits
    pass
