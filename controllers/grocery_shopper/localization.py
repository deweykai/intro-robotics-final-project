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
        rad = ((math.atan2(n[0], -n[2])))
        pose_theta = rad


positioning = GPS()


def init():
    pass


def update():
    positioning.update()

    print(pose_x, pose_y)
