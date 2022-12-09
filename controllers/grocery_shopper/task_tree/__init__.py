"""Automation behaviour tree

This system builds a behaviour tree to make the robot collect objects.

There are three high level branches of the tree:
- grab object: The robot sees an object close to it and takes steps to
put it in its the basket.
- goto object: The robot sees an object far away and drives to an
appropriate position to trigger the previous branch.
- wander: just drive around to find objects.

The `wander` branch can be used on its own to help generate a map of the world.

This module create a bunch of different behaviour nodes that it needs to
function.
"""

from .drive_to import DriveTo
from .wait import Timer
from . import find_object
from py_trees.composites import *
import py_trees as pyt
from py_trees.common import Status
from .face_towards import FaceTowards
from .arms import SetArms
from .gripper import SetGripper
from .fowards import DriveForwards
import bus
import numpy as np

AILE_WIDTH = 4

pose_x, pose_y, pose_theta = 0, 0, 0
autonomous = False
object_dist = 0


@bus.subscribe('/bot/pose', np.ndarray)
def gps_data(data):
    global pose_x, pose_y, pose_theta
    pose_x, pose_y, pose_theta = data


def in_front_of_object():
    # this should only be called once an object is detected and find_object
    # has saved that position
    object_pos = find_object.object_location
    # the object is shelf, the shelf can be left or right of the object, we want to be in front.
    # assume we cannot see an object from behind the shelf
    if object_pos[1] > pose_y:
        target_y = object_pos[1] - AILE_WIDTH / 2
    if object_pos[1] < pose_y:
        target_y = object_pos[1] + AILE_WIDTH / 2

    target_x = object_pos[0]
    return [target_x, target_y]


wander_positions = [
    [-5, 5.65],
    [13.1, 5.65],
    [13, -5.6],
    [-5.15, -5.65],
    [-4.8, -1.9],
    [13, -1.9],
    [13, 2],
    [-5, 2],
]
drive_index = 0


class UpdateWanderPosition(pyt.behaviour.Behaviour):
    def update(self):
        global drive_index
        drive_index = (drive_index + 1) % len(wander_positions)
        return Status.SUCCESS


wander_tree = Sequence(children=[
    DriveTo(get_position=lambda: wander_positions[drive_index]),
    UpdateWanderPosition(),
])


close_range_grab_object = Sequence(name='grab close range', children=[
    # check object position again
    find_object.FindObject(close_range=True),
    FaceTowards(get_target=lambda: find_object.object_location),

    # try to fix alignment
    DriveForwards(speed=1.0),
    Timer(ms=1_000),
    FaceTowards(get_target=lambda: find_object.object_location),
    Timer(ms=1_000),


    # ready arm and gripper to grab target
    SetArms(get_target=lambda: find_object.object_location),
    SetGripper(open_state=True),
    Timer(ms=10_000),


    # drive into object, make sure to face object
    DriveForwards(speed=1.0, cond=lambda: object_dist < 1.3),
    FaceTowards(get_target=lambda: find_object.object_location),

    DriveForwards(speed=1.0, cond=lambda: object_dist < 1.1),
    FaceTowards(get_target=lambda: find_object.object_location),

    DriveForwards(speed=1.0, cond=lambda: object_dist < 0.9),
    FaceTowards(get_target=lambda: find_object.object_location),

    DriveForwards(speed=1.0, cond=lambda: object_dist < 0.7),

    # grab object
    DriveForwards(speed=0.0),
    SetGripper(open_state=False),
    Timer(ms=1_000),

    # back away from shelf
    DriveForwards(speed=-1.0),
    Timer(ms=10_000),

    # put object in basket
    DriveForwards(speed=0.0),
    SetArms(state='pre-basket'),
    Timer(ms=5_000),

    SetArms(state='basket'),
    Timer(ms=5_000),

    SetArms(state='post-basket'),
    Timer(ms=3_000),
    SetGripper(open_state=True),
    Timer(ms=5_000),
    SetArms(state='standby'),
    Timer(ms=5_000),
])

long_range_grab_object = Sequence(name='grab long range', children=[
    # object_visible,
    find_object.FindObject(close_range=False),
    # drive to object
    DriveTo(get_position=in_front_of_object),
    # face towards target
    FaceTowards(get_target=lambda: find_object.object_location),
])

root = Selector(children=[
    close_range_grab_object,
    long_range_grab_object,
    wander_tree
])

root.setup()


@bus.subscribe('/bot/cmd_tick', int)
def tick_root(_):
    global object_dist
    if autonomous:
        root.tick_once()
        dx = find_object.object_location[0] - pose_x
        dy = find_object.object_location[1] - pose_y
        object_dist = np.linalg.norm([dx, dy])


@bus.subscribe('/bot/cmd_auto', bool)
def auto_mode(toggle_auto):
    global autonomous
    autonomous = toggle_auto
