import py_trees as pyt
from py_trees.common import Status
from robot import timestep


class Timer(pyt.behaviour.Behaviour):
    def __init__(self, ms=100, name='Timer'):
        super().__init__(name)
        self.passed = 0
        self.ms = ms

    def initialise(self):
        self.passed = 0
        return super().initialise()

    def update(self):
        self.passed += timestep
        if self.passed < self.ms:
            return Status.RUNNING
        else:
            return Status.SUCCESS
