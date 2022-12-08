"""grocery controller."""

# Nov 2, 2022

import utils.logger_config as _
import bus
from robot import *
import sensors.gps as _
import sensors.lidar as _
import sensors.camera as _
import motors.wheel as _
import motors.arm as _
import motors.gripper as _
import service.teleop as _
import service.mapping as _
# import service.odometry as _
import task_tree as _

# Main Loop
tick_cmd_publisher = bus.Publisher('/bot/cmd_tick', int)

while robot.step(timestep) != -1:
    tick_cmd_publisher.publish(timestep)
