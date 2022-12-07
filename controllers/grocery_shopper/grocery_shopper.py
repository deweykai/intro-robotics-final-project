"""grocery controller."""

# Nov 2, 2022

import bus
from robot import *
# import mapping
# import vision
# import manipulation
# import navigation
import sensors.gps as _
import sensors.lidar as _
import motors.wheel as _
import teleop as _
import mapping as _
# import task_tree as _
import logging
import sys


class ColorFormatter(logging.Formatter):
    grey = "\x1b[38;20m"
    green = "\x1b[32m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = "[%(asctime)s] [%(levelname)8s] - %(message)s (%(filename)s:%(lineno)d)"

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: green + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


console_handler = logging.StreamHandler(sys.stdout)
console_handler.setLevel(logging.INFO)
console_handler.setFormatter(ColorFormatter())

file_handler_debug = logging.FileHandler('controller_debug.log', mode='w')
file_handler_debug.setFormatter(logging.Formatter(
    "[%(asctime)s] [%(levelname)8s] - %(message)s (%(filename)s:%(lineno)d)"))
file_handler_debug.setLevel(logging.DEBUG)


file_handler_info = logging.FileHandler('controller_info.log', mode='w')
file_handler_info.setFormatter(logging.Formatter(
    "[%(asctime)s] [%(levelname)8s] - %(message)s (%(filename)s:%(lineno)d)"))
file_handler_info.setLevel(logging.INFO)

logging.basicConfig(level=logging.INFO, handlers=[
                    console_handler, file_handler_info, file_handler_debug])


# init modules
# robot.step(timestep) != -1
# localization.init()
# vision.init()
# mapping.init()
# navigation.init()
# manipulation.init()
# print(timestep)

# Main Loop

tick_cmd_publisher = bus.Publisher('/bot/cmd_tick', int)

while robot.step(timestep) != -1:
    tick_cmd_publisher.publish(timestep)
#     vision.update()
#     mapping.update()
#     navigation.update()
#     manipulation.update()
