import numpy as np
import matplotlib.pyplot as plt
import math
import random
from service import mapping
from scipy.signal import convolve2d

import logging
logger = logging.getLogger(__name__)

CONV_SIZE = 15

DEBUG = True

###############################################################################
# Base Code
###############################################################################


class Node:
    """
    Node for RRT Algorithm. This is what you'll make your graph with!
    """

    def __init__(self, pt, parent=None):
        self.point = pt  # n-Dimensional point
        self.parent = parent  # Parent node
        # List of points along the way from the parent node (for edge's collision checking)
        self.path_from_parent = []


def get_state_is_valid(map_data):
    def state_is_valid(state):
        '''
        Function that takes an n-dimensional point and checks if it is within the bounds and not inside the obstacle
        :param state: n-Dimensional point
        :return: Boolean whose value depends on whether the state/point is valid or not
        '''
        state_bounds = np.array([[0, 360], [0, 360]])
        for dim in range(state_bounds.shape[0]):
            if state[dim] < state_bounds[dim][0]:
                return False
            if state[dim] >= state_bounds[dim][1]:
                return False

        if map_data[int(state[1])][int(state[0])] == 1:
            return False  # obstacle

        return True

    return state_is_valid


def get_random_valid_vertex(state_is_valid, bounds):
    '''
    Function that samples a random n-dimensional point which is valid (i.e. collision free and within the bounds)
    :param state_valid: The state validity function that returns a boolean
    :param bounds: The world bounds to sample points from
    :return: n-Dimensional point/state
    '''
    vertex = None
    while vertex is None:  # Get starting vertex
        pt = np.random.rand(bounds.shape[0]) * \
            (bounds[:, 1]-bounds[:, 0]) + bounds[:, 0]
        if state_is_valid(pt):
            vertex = pt
    return vertex

###############################################################################
# END BASE CODE
###############################################################################


def get_nearest_vertex(node_list, q_point):
    '''
    Function that finds a node in node_list with closest node.point to query q_point
    :param node_list: List of Node objects
    :param q_point: n-dimensional array representing a point
    :return Node in node_list with closest node.point to query q_point
    '''
    return sorted(node_list, key=lambda n: np.linalg.norm(n.point - q_point))[0]


def steer(from_point, to_point, delta_q):
    '''
    :param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
    :param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
    :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
    :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
    '''
    # cap the length between `from_point` to `to_point` to delta_q
    vect = to_point - from_point
    mag = np.linalg.norm(vect)
    if mag > delta_q:
        vect = (vect / mag) * delta_q
        to_point = vect + from_point

    path = np.linspace(from_point, to_point, 10)
    return path


def check_path_valid(path, state_is_valid):
    '''
    Function that checks if a path (or edge that is made up of waypoints) is collision free or not
    :param path: A 1D array containing a few (10 in our case) n-dimensional points along an edge
    :param state_is_valid: Function that takes an n-dimensional point and checks if it is valid
    :return: Boolean based on whether the path is collision free or not
    '''

    for point in path:
        if not state_is_valid(point):
            return False
    return True


def rrt(state_bounds, state_is_valid, starting_point, goal_point, K, delta_q):
    '''
    Implement the RRT algorithm here, making use of the provided state_is_valid function.
    RRT algorithm.
    If goal_point is set, your implementation should return once a path to the goal has been found 
    (e.g., if q_new.point is within 1e-5 distance of goal_point), using k as an upper-bound for iterations. 
    If goal_point is None, it should build a graph without a goal and terminate after k iterations.

    :param state_bounds: matrix of min/max values for each dimension (e.g., [[0,1],[0,1]] for a 2D 1m by 1m square)
    :param state_is_valid: function that maps states (N-dimensional Real vectors) to a Boolean (indicating free vs. forbidden space)
    :param starting_point: Point within state_bounds to grow the RRT from
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param k: Number of points to sample
    :param delta_q: Maximum distance allowed between vertices
    :returns List of RRT graph nodes
    '''
    # rrt
    threshold = 1e-5
    # Add Node at starting point with no parent
    node_list = [Node(starting_point, parent=None)]
    for _ in range(K):
        target = np.array(get_random_valid_vertex(
            state_is_valid, state_bounds))

        # if the goal point is close enough to a node, return early
        if goal_point is not None:
            if np.linalg.norm(get_nearest_vertex(node_list, goal_point).point - goal_point) < threshold:
                return node_list
            if random.random() < 0.05:
                target = goal_point

        nearest = get_nearest_vertex(node_list, target)
        path = steer(nearest.point, target, delta_q)

        if check_path_valid(path, state_is_valid):
            new_node = Node(path[-1], parent=nearest)
            new_node.path_from_parent = path
            node_list.append(new_node)

    return node_list


def smooth_path(path, cspace):
    if len(path) < 2:
        return []

    current = path.pop(0)
    previous = None
    next = path.pop(0)
    new_path = [current]

    while len(path) > 1:
        previous = next
        next = path.pop(0)
        dist = np.linalg.norm(current - next)
        line = np.linspace(current, next, int(dist+1))
        for x, y in line:
            if cspace[int(y)][int(x)] == 1:
                new_path.append(previous)
                current = previous
                logger.debug(f'hit at {next}')
                break

    new_path.append(path.pop(0))
    logger.debug(new_path)
    return new_path


def plan_path(start_pos, target_pos: list[float]):
    logger.debug(f'world target pos: {target_pos}')

    end_p = mapping.coords_world_to_map((target_pos[0], target_pos[1]))
    end_p = np.array(end_p)
    start_p = mapping.coords_world_to_map((start_pos[0], start_pos[1]))
    start_p = np.array(start_p)
    logger.debug(f'plan path from {start_p} to {end_p}')

    adj_map = convolve2d(mapping.map_data, np.ones((CONV_SIZE, CONV_SIZE)), mode='same',
                         boundary='fill', fillvalue=1)
    adj_map = (adj_map > (CONV_SIZE**2 * 0.01)) * 1

    K = 1_000  # Feel free to adjust as desired
    bounds = np.array([[0, 360], [0, 360]])
    state_is_valid = get_state_is_valid(adj_map)

    nodes = rrt(bounds, state_is_valid, start_p,
                end_p, K, np.linalg.norm(bounds) / 20)

    path = []

    goal_node = get_nearest_vertex(nodes, end_p)

    while goal_node is not None:
        path.append(goal_node.point)
        goal_node = goal_node.parent

    path = list(reversed(path))
    #path = smooth_path(path, adj_map)

    fig, axes = plt.subplots(nrows=2)

    if DEBUG:
        axes[0].imshow(adj_map)
        x = np.array([a[0] for a in path])
        y = np.array([a[1] for a in path])
        axes[0].scatter(x, y)

    path = smooth_path(path, adj_map)

    if DEBUG:
        axes[1].imshow(adj_map)
        x = np.array([a[0] for a in path])
        y = np.array([a[1] for a in path])
        axes[1].scatter(x, y)
        plt.show()

    # ignore the starting waypoint
    waypoints = [mapping.coords_map_to_world(pos) for pos in path][1:]
    return waypoints


def visualize_2D_graph(state_bounds, obstacles, nodes, goal_point=None, filename=None):
    '''
    Function to visualise the 2D world, the RRT graph, path to goal if goal exists
    :param state_bounds: Array of min/max for each dimension
    :param obstacles: Locations and radii of spheroid obstacles
    :param nodes: List of vertex locations
    :param goal_point: Point within state_bounds to target with the RRT. (OPTIONAL, can be None)
    :param filename: Complete path to the file on which this plot will be saved
    :return: None
    '''
    fig = plt.figure()
    plt.xlim(state_bounds[0, 0], state_bounds[0, 1])
    plt.ylim(state_bounds[1, 0], state_bounds[1, 1])

    # plotting obstacles
    x = [item[0] for item in obstacles]
    y = [item[1] for item in obstacles]
    colors = np.random.rand(len(x))
    plt.scatter(x, y, s=15, c=colors, alpha=0.5)

    goal_node = None
    for node in nodes:
        if node.parent is not None:
            node_path = np.array(node.path_from_parent)
            plt.plot(node_path[:, 0], node_path[:, 1], '-b')
        # The goal may not be on the RRT so we are finding the point that is a 'proxy' for the goal
        if goal_point is not None and np.linalg.norm(node.point - np.array(goal_point)) <= 1e-5:
            goal_node = node
            plt.plot(node.point[0], node.point[1], 'k^')
        else:
            plt.plot(node.point[0], node.point[1], 'ro')

    plt.plot(nodes[0].point[0], nodes[0].point[1], 'ko')

    if goal_node is not None:
        cur_node = goal_node
        while cur_node is not None:
            if cur_node.parent is not None:
                node_path = np.array(cur_node.path_from_parent)
                plt.plot(node_path[:, 0], node_path[:, 1], '-y')
                cur_node = cur_node.parent
            else:
                break

    if goal_point is not None:
        plt.plot(goal_point[0], goal_point[1], 'gx')

    if filename is not None:
        fig.savefig(filename)
    else:
        plt.show()


if __name__ == "__main__":
    K = 250  # Feel free to adjust as desired
    bounds = np.array([[0, 360], [0, 360]])

    # starting_point = get_random_valid_vertex(state_is_valid, bounds)
    goal_point = get_random_valid_vertex(state_is_valid, bounds)
    # while np.linalg.norm(starting_point - goal_point) < np.linalg.norm(bounds/2.):
    #     starting_point = get_random_valid_vertex(state_is_valid, bounds)
    #     goal_point = get_random_valid_vertex(state_is_valid, bounds)

    #nodes = rrt(bounds, state_is_valid, starting_point, goal_point, K, np.linalg.norm(bounds/10.))
    nodes = plan_path(goal_point, bounds)

    obstacles = []
    for row in range(len(map_data)):
        for col in range(len(map_data)):
            if map_data[row][col] == 1:  # obstacle
                obstacles.append([col, row])

    visualize_2D_graph(bounds, obstacles, nodes,
                       goal_point, 'rrt_goal_run2.png')
