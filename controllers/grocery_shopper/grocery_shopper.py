"""Grocery Shopper Controller

This script is the entry point for the grocery shopper controller.
This script does 2 things. It loads other modules, and then runs the
main tick event loop which drives the other modules.

This controller requires the following libraries to be installed:
- numpy
- py_trees

This controller can only be run within webots.

This controller takes somewhere between 20-40 minutes to collect all
objects (or drop them on the floor).

Issues:
The sometimes a cube falls on the floor. In that case we give up on it.

Sometimes a cube gets stuck in the grabber. It doesn't cause a problem
since it falls out eventually, but it usually falls out onto the floor.

Rarely the robot gets stuck driving to a waypoint and goes in circles.
You can either wait a while or go into manual mode to move the robot away
which usually fixes the problem.

Rarely the robots arm gets stuck on upper shelf, lifting the wheels
off the ground so the robot can't move. Press "1" to move the arm up which
should fix the problem.

There is a cube floating in the void outside the market. Ignore it.
"""

from robot import *

# This module configures some fancy logging.
import utils.logger_config as _

# The bus module provides the framework that all modules use to
# communicate with each other.
import bus

# Sensors
import sensors.gps as _
import sensors.lidar as _
import sensors.camera as _

# Motors
import motors.wheel as _
import motors.arm as _
import motors.gripper as _

# Services
import service.teleop as _
import service.mapping as _
import service.identify_object as _
#import service.odometry as _

# Automation
import task_tree as _

# Main Loop
tick_cmd_publisher = bus.Publisher('/bot/cmd_tick', int)

while robot.step(timestep) != -1:
    tick_cmd_publisher.publish(timestep)
