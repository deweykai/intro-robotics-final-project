import py_trees as pyt
from py_trees.common import Status


class Timer(pyt.behaviour.Behaviour):
    def __init__(self, ticks=100, name='Timer'):
        super().__init__(name)
        self.wait_ticks = ticks

    def initialise(self):
        self.tick = 0
        return super().initialise()

    def update(self):
        if self.tick < self.wait_ticks:
            return Status.RUNNING
        else:
            return Status.SUCCESS
