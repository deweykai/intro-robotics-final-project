from .drive_to import DriveTo
from .wait import Timer
from . import find_object
from py_trees.composites import *
from .face_towards import FaceTowards
import py_trees as pyt
import localization as loc

AILE_WIDTH = 4


def in_front_of_object():
    # this should only be called once an object is detected and find_object
    # has saved that position
    object_pos = find_object.object_location
    # the object is shelf, the shelf can be left or right of the object, we want to be in front.
    # assume we cannot see an object from behind the shelf
    if object_pos[1] > loc.pose_y:
        target_y = object_pos[1] - AILE_WIDTH / 2
    if object_pos[1] < loc.pose_y:
        target_y = object_pos[1] + AILE_WIDTH / 2

    target_x = object_pos[0]
    return [target_x, target_y]


def create_root():
    wander_tree = Sequence(children=[
        DriveTo(get_position=lambda: [-5, 5.65]),
        DriveTo(get_position=lambda: [13.1, 5.65]),
        DriveTo(get_position=lambda: [13, -5.6]),
        DriveTo(get_position=lambda: [-5.15, -5.65]),
        DriveTo(get_position=lambda: [-4.8, -1.9]),
        DriveTo(get_position=lambda: [13, -1.9]),
        DriveTo(get_position=lambda: [13, 2]),
        DriveTo(get_position=lambda: [-5, 2]),
    ])

    retrieve_object_tree = Sequence(children=[
        find_object.FindObject(),
        # object_visible,
        Parallel(children=[
            DriveTo(get_position=in_front_of_object),
            # drive_infront_of_object
            # position_arm
        ]),
        FaceTowards(get_target=lambda: find_object.object_location),
        Timer(ms=5000),
        # grab object
    ])

    root = Selector(children=[
        retrieve_object_tree,
        wander_tree
    ])

    root = FaceTowards(lambda: [-12, -6])

    return root
