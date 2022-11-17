"""Planning and navigation

This module does path planning and decision making.

This file provides:
    * waypoints[] - a list of (x,y) coordinates to the robots destination

This module depends:
    * mapping - map data to plan a path
"""
import numpy as np
import mapping
import localization as loc
from scipy.signal import convolve2d
import matplotlib.pyplot as plt
import logging

logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)

CONV_SIZE = 20

# display waypoints on map
DEBUG = True


def a_star(map_data, start, end):
    '''
    :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
    :param start: A tuple of indices representing the start cell in the map
    :param end: A tuple of indices representing the end cell in the map
    :return: A list of tuples as a path from the given start to the given end in the given maze
    '''

    def h(pos, target):
        x, y = pos
        x2, y2 = target
        return np.sqrt((x2 - x)**2 + (y2-y)**2)

    # dijkstra's algorithm find shortest path from start to all nodes
    dist = np.ones(map_data.shape) * np.Infinity
    logger.debug(f'map shape {dist.shape}')
    logger.debug(f'start position {start}')
    dist[start[1]][start[0]] = 0
    unvisited = {start: h(start, end)}
    prev = {start: None}  # each node will have prev node to keep track

    while len(unvisited) > 0:
        node, _ = sorted(unvisited.items(), key=lambda item: item[1])[
            0]  # sort by h(node)
        del unvisited[node]
        if node == end:
            break

        # getting location of node
        index_x, index_y = node

        adj = [
            (index_x-1, index_y),  # north
            (index_x+1, index_y),  # south
            (index_x, index_y-1),  # east
            (index_x, index_y+1),  # west
        ]
        for x, y in adj:
            if (y >= 0 and y < len(map_data) and x >= 0 and x < len(map_data[y])) and map_data[y][x] == 0:
                new_distance = dist[index_y][index_x] + 1
                if new_distance < dist[y][x]:
                    dist[y][x] = new_distance
                    unvisited[(x, y)] = new_distance + h((x, y), end)
                    prev[(x, y)] = node

    # get path from start to end
    path = []
    temp = end
    try:
        while temp != start and temp != None:
            path.append(temp)
            temp = prev[temp]
    except KeyError:
        pass

    path = list(reversed(path))
    return path


def smooth_path(path, cspace):
    if len(path) < 2:
        return []

    current = path[0]
    new_path = [current]

    for next in path[1:-1]:
        line = np.linspace(current, next, 10)
        for x, y in line:
            if cspace[int(y)][int(x)] == 1:
                new_path.append(next)
                current = next
                logger.debug(f'hit at {next}')
                break

    new_path.append(path[-1])
    logger.debug(new_path)
    return new_path


def plan_path(target_pos: list[float]):
    logger.debug(f'world target pos: {target_pos}')

    end_p = mapping.coords_world_to_map((target_pos[0], target_pos[1]))
    start_p = mapping.coords_world_to_map((loc.pose_x, loc.pose_y))
    logger.debug(f'plan path from {start_p} to {end_p}')

    adj_map = convolve2d(mapping.map_data, np.ones((CONV_SIZE, CONV_SIZE)), mode='same',
                         boundary='fill', fillvalue=1)
    adj_map = (adj_map > (CONV_SIZE**2 * 0.05)) * 1
    plt.imshow(mapping.map_data)
    plt.show()
    plt.imshow(adj_map)
    plt.show()

    path = a_star(adj_map, start_p, end_p)
    path = smooth_path(path, adj_map)

    if DEBUG:
        plt.imshow(adj_map)
        plt.gca().invert_yaxis()
        x = np.array([a[0] for a in path])
        y = np.array([a[1] for a in path])
        plt.scatter(x, y)
        plt.show()

    waypoints = [mapping.coords_map_to_world(pos) for pos in path]
    logger.info('waypoints calculated')
    return waypoints


def init():
    """Initialize navigation module"""

    pass


def update():
    """Update hook for navigation module"""

    pass
