"""Webots mapping

This module maintains a map of the webots world to be used by other modules.

This file provides variables:
    * map_data[a][b] - 2d array, 1 for obstacles and 0 for free space
    * map_width - width of map_data
    * map_height - height of map_data
    * dirty - did map change this update cycle

This file provides functions:
    * coords_map_to_world((a, b)): (x, y) - converts map indexes to world coordinates
    * coords_world_to_map((x, y)): (a, b) - converts world coordinates to map indexes

This module depends on:
    * vision - uses camera to generate map
    * localization - to orient robot in map
"""

import numpy as np
import matplotlib.pyplot as plt
from robot import display, DISPLAY_DIM
from vision import get_lidar_readings
import localization as loc

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

    def update(self):
        """Update internal raw map data"""
        readings = get_lidar_readings()
        try:
            readings = [coords_world_to_map(pos) for pos in readings]
        except Exception as e:
            print(e)

        for x, y in readings:
            # gray scale lidar readings
            g = self.raw_map[y][x] + 5e-3
            g = self.raw_map[y][x] = max(0, min(1, g))

            color = int((int(g * 254) << 16) +
                        (int(g * 254) << 8) + int(g * 254))

            display.setColor(color)
            # display is adjusted so the orientation matches viewport window
            display.drawPixel(DISPLAY_DIM - y, DISPLAY_DIM - x)

        try:
            robot_x, robot_y = coords_world_to_map((loc.pose_x, loc.pose_y))
            # Draw the robot's current pose on the 360x360 display
            display.setColor(int(0xFF0000))
            # display is adjusted so the orientation matches viewport window
            display.drawPixel(DISPLAY_DIM - robot_y, DISPLAY_DIM - robot_x)
        except Exception as e:
            print(e)

    def update_map_data(self):
        """Process raw_map then save to map_data"""

        raw_map = self.raw_map.copy()

        global map_data, dirty
        map_data = (raw_map > 0.7) * 1
        dirty = True

    def load(self):
        """Load raw map data from `raw_map.npy`"""

        print('Loading map...')
        self.raw_map = np.load('raw_map.npy')
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

        print('Saving map...')
        np.save('raw_map.npy', self.raw_map)


mapper = ManualMapper()


def init():
    """Initialize mapping module"""

    mapper.load()


def update():
    """Update hook for mapping module"""

    mapper.update()
