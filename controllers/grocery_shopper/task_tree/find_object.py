import py_trees as pyt
from py_trees.common import Status
import localization as loc
import logging

object_location = [0, 0]
# debug only
found_once = False


class FindObject(pyt.behaviour.Behaviour):
    def update(self):
        global object_location

        global found_once
        # hard code object
        if found_once:
            return Status.FAILURE
        if loc.pose_y > 5:
            object_location = [3.5, 7.17, 1.07]
            found_once = True
            return Status.SUCCESS

        # find tree
        return Status.FAILURE
