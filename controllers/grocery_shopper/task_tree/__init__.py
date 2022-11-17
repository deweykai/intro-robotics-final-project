from .drive_to import DriveTo
from . import find_object
from py_trees.composites import *


def create_root():
    wander_tree = Sequence(children=[
        DriveTo(get_position=lambda: [-5, 5.65]),
        DriveTo(get_position=lambda: [13.1, 5.65]),
        DriveTo(get_position=lambda: [13, -5.6]),
        DriveTo(get_position=lambda: [-5.15, -5.65]),
    ])

    retrieve_object_tree = Sequence(children=[
        find_object.FindObject(),
        # object_visible,
        Parallel(children=[
            # drive_infront_of_object
            # position_arm
        ]),
        # grab object
    ])

    return Selector(children=[
        retrieve_object_tree,
        wander_tree
    ])
