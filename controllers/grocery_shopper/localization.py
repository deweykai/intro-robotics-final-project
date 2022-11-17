"""Robot localization

This module provide information of the spatial position of the robot.
    
This file provides variables:
    * pose_x - x position of robot
    * pose_y - y position of robot
    * pose_theta - ground rotation of robot
"""


import math
from robot import gps, compass

# Odometry
pose_x = 0
pose_y = 0
pose_theta = 0


class GPS:
    def update(self):
        global pose_x, pose_y, pose_theta
        pose_x = gps.getValues()[0]
        pose_y = gps.getValues()[1]

        n = compass.getValues()

        # F compass coords are different from lab 5
        rad = ((math.atan2(n[0], n[1])))
        pose_theta = rad


positioning = GPS()


def init():
    """Initialize localization module"""
    positioning.update()

    pass


def update():
    """Update hook for localization module"""

    positioning.update()
