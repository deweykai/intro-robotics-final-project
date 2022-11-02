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

map_width = np.nan
map_height = np.nan
map_data = np.zeros((map_width, map_height))
dirty = False


def init():
    """Initialize mapping module"""

    pass


def update():
    """Update hook for mapping module"""

    global dirty
    dirty = False
    pass
