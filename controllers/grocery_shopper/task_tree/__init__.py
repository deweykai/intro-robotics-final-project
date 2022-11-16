from .drive_to import DriveTo
from py_trees.composites import *


def create_root():
    wander_tree = Sequence(children=[
        DriveTo(target_pos=[-5, 5.65]),
        DriveTo(target_pos=[13.1, 5.65]),
        DriveTo(target_pos=[13, -5.6]),
        DriveTo(target_pos=[-5.15, -5.65]),
    ])

    return wander_tree
