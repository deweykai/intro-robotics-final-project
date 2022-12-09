"""Webots mapping

This module maintains a map of the webots world to be used by other modules.
It also provides functionality to generate a map when the robot wanders
around the map.

This file provides variables:
    * map_data[a][b] - 2d array, 1 for obstacles and 0 for free space
    * map_width - width of map_data
    * map_height - height of map_data

This file provides functions:
    * coords_map_to_world((a, b)): (x, y) - converts map indexes to world coordinates
    * coords_world_to_map((x, y)): (a, b) - converts world coordinates to map indexes
"""

import logging
import matplotlib.pyplot as plt
import numpy as np
import bus

pose_x, pose_y, pose_theta = 0, 0, 0


def gps_data(data):
    global pose_x, pose_y, pose_theta
    pose_x, pose_y, pose_theta = data


bus.Subscriber('/bot/pose', np.ndarray, gps_data)


logger = logging.getLogger(__name__)

LOCKED = True  # prevent overwriting saved map file
DISPLAY_DIM = 360

WORLD_MIN_X = -15
WORLD_MAX_X = 15
WORLD_MIN_Y = -8
WORLD_MAX_Y = 8


map_width = DISPLAY_DIM
map_height = DISPLAY_DIM
map_data = np.zeros((map_width, map_height))
dirty = False


def coords_world_to_map(pos):
    x, y = pos
    new_x = int((x / (WORLD_MAX_X - WORLD_MIN_X) + 0.5) * DISPLAY_DIM)
    new_y = int((y / (WORLD_MAX_X - WORLD_MIN_X) + 0.5) * DISPLAY_DIM)
    if new_x < 0 or new_x >= DISPLAY_DIM:
        raise Exception('x out of bounds')
    if new_y < 0 or new_y >= DISPLAY_DIM:
        raise Exception('y out of bounds')
    return new_x, new_y


def coords_map_to_world(map_pos):
    x, y = map_pos
    new_x = ((x / DISPLAY_DIM) - 0.5) * (WORLD_MAX_X - WORLD_MIN_X)
    new_y = ((y / DISPLAY_DIM) - 0.5) * (WORLD_MAX_X - WORLD_MIN_X)
    return new_x, new_y


class ManualMapper:
    def __init__(self):
        self.raw_map = np.zeros(map_data.shape)

    def update(self, readings):
        """Update internal raw map data"""
        from robot import display
        try:
            robot_x, robot_y = coords_world_to_map((pose_x, pose_y))
            # Draw the robot's current pose on the 360x360 display
            display.setColor(int(0xFF0000))
            # display is adjusted so the orientation matches viewport window
            display.drawPixel(DISPLAY_DIM - robot_y, DISPLAY_DIM - robot_x)
        except Exception as e:
            logger.error(e)
        try:
            readings = [coords_world_to_map(pos) for pos in readings]
        except Exception as e:
            logger.error(e)

        for x, y in readings:
            # gray scale lidar readings
            g = self.raw_map[y][x] + 5e-3
            g = self.raw_map[y][x] = max(0, min(1, g))

            color = int((int(g * 254) << 16) +
                        (int(g * 254) << 8) + int(g * 254))

            display.setColor(color)
            # display is adjusted so the orientation matches viewport window
            display.drawPixel(DISPLAY_DIM - y, DISPLAY_DIM - x)

    def update_map_data(self):
        """Process raw_map then save to map_data"""

        raw_map = self.raw_map.copy()

        global map_data, dirty
        map_data = (raw_map > 0.7) * 1
        dirty = True

    def load(self):
        """Load raw map data from `raw_map.npy`"""

        from robot import display
        logger.info('Loading map...')
        self.raw_map = np.load('raw_map.npy')
        # plt.imshow(self.raw_map)
        # plt.show()
        for y in range(DISPLAY_DIM):
            for x in range(DISPLAY_DIM):
                g = self.raw_map[y][x]
                color = int((int(g * 254) << 16) +
                            (int(g * 254) << 8) + int(g * 254))

                display.setColor(color)
                # display is adjusted so the orientation matches viewport window
                display.drawPixel(DISPLAY_DIM - y, DISPLAY_DIM - x)

        self.update_map_data()

    def save(self):
        """Save internal raw map data to file `raw_map.npy`"""

        if not LOCKED:
            logger.info('Saving map...')
            np.save('raw_map.npy', self.raw_map)
        else:
            logger.warning(
                'Map cannot save because the mapping module is locked.')


mapper = ManualMapper()


@bus.subscribe('/bot/cmd_map', str)
def cmd_map(cmd):
    if cmd == 'load':
        mapper.load()

    if cmd == 'save':
        mapper.save()


@bus.subscribe('/bot/sensor/lidar', list)
def update_map(readings):
    mapper.update(readings)
