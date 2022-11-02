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
vision.init()
localization.init()
mapping.init()
navigation.init()
manipulation.init()

# Main Loop
while robot.step(timestep) != -1:
    vision.update()
    localization.update()
    mapping.update()
    navigation.update()
    manipulation.update()
