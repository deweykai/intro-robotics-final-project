import py_trees as pyt
from py_trees.common import Status

object_location = [0, 0]


class FindObject(pyt.behaviour.Behaviour):
    def update(self):
        # find tree
        return Status.FAILURE
