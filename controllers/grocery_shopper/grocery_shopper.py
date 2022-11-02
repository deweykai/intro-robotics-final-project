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
mapping.init()
localization.init()
vision.init()
manipulation.init()
navigation.init()

# Main Loop
while robot.step(timestep) != -1:
    mapping.update()
    localization.update()
    vision.update()
    manipulation.update()
    navigation.update()
