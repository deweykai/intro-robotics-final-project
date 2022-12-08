from robot import robot, timestep
import bus
import numpy as np

pub = bus.Publisher('/bot/pose', np.ndarray)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)


@bus.subscribe('/bot/cmd_tick', int)
def update(_):
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]

    n = compass.getValues()

    # F compass coords are different from lab 5
    rad = ((np.arctan2(n[0], n[1])))
    pose_theta = rad
    pub.publish(np.array([pose_x, pose_y, pose_theta]))


#bus.inspect('/bot/pose', np.ndarray)
