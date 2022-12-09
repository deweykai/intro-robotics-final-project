"""Grocery Shopper Controller

This script is the entry point for the grocery shopper controller.
This script does 2 things. It loads other modules, and then runs the
main tick event loop which drives the other modules.

This controller requires the following libraries to be installed:
- numpy
- py_trees

This controller can only be run within webots.
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
