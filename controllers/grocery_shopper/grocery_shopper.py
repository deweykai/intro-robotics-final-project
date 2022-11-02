"""grocery controller."""

# Nov 2, 2022

from robot import *
import mapping
import localization
import vision
import manipulation
import navigation


# init modules
robot.step(timestep) != -1
localization.init()
vision.init()
mapping.init()
navigation.init()
manipulation.init()

# Main Loop
while robot.step(timestep) != -1:
    localization.update()
    vision.update()
    mapping.update()
    navigation.update()
    manipulation.update()
