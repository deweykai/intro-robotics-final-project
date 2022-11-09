"""Planning and navigation

This module does path planning and decision making.

This file provides:
    * waypoints[] - a list of (x,y) coordinates to the robots destination

This module depends:
    * mapping - map data to plan a path
"""
import numpy as np

# TODO: Copy A* path finding from Lab5
# TODO: Implement RRT path finding

global map


def WIP():
    map = np.load("map.npy")
    dist = np.ones(map.shape) * np.Infinity
    dist[start[0]][start[1]] = 0
    unvisited = {start: h(start, end)}
    prev = {start: None}

    while len(unvisited) > 0:
        node, _ = sorted(unvisited.items(), key=lambda item: item[1])[0]
        del unvisited[node]
        if node == end:
            break

        # getting location of node
        index1 = node[0]  # x
        index2 = node[1]  # y

        adj = [
            (index1 - 1, index2),  # north
            (index1 + 1, index2),  # south
            (index1, index2 - 1),  # east
            (index1, index2 + 1),  # west
        ]
        for x, y in adj:
            if (x >= 0 and x < len(map) and y >= 0 and y < len(map[x])) and map[x][
                y
            ] == 0:
                new_distance = dist[index1][index2] + 1
                if new_distance < dist[x][y]:
                    dist[x, y] = new_distance
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


def init():
    """Initialize navigation module"""

    pass


def update():
    """Update hook for navigation module"""

    pass
