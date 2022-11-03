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

map_width = 10
map_height = 10
map_data = np.zeros((map_width, map_height))
dirty = False


class ManualMapper:
    def __init__(self):
        self.raw_map = np.zeros(map_data.shape)

    def update(self):
        """Update internal raw map data"""
        # TODO: update internal map data
        # TODO: show internal map on robot display

    def update_map_data(self):
        """Process raw_map then save to map_data"""

        raw_map = self.raw_map.copy()

        # TODO: save map data

        global map_data, dirty
        map_data = raw_map
        dirty = True

    def load(self):
        """Load raw map data from `raw_map.npy`"""

        print('Loading map...')
        # TODO: load map from file

    def save(self):
        """Save internal raw map data to file `raw_map.npy`"""

        print('Saving map...')
        # TODO: save map from file


mapper = ManualMapper()


def init():
    """Initialize mapping module"""

    pass


def update():
    """Update hook for mapping module"""

    mapper.update()
