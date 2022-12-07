import robot
import bus


@bus.subscribe('/bot/cmd_gripper', bool)
def cmd_gripper(open_state):
    pass
