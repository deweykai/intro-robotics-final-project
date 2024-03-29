"""Robot definitions

This module contains variables used by the robot.
This module also initializes various robot devices
to be used by the rest of the controller.

This module provides devices:
    * robot - instance of webots robot controller
    * timestep - webots timestep
    * robot_parts - a dictionary of robot joints including head, torso, arm, wheel, and grippers.
    * left_gripper_enc - left gripper encoder
    * right_gripper_enc - right gripper encoder
    * display - webots display device
"""

from controller import Robot

# Initialization
print("=== Initializing Grocery Shopper...")

# Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633  # [m/s]
AXLE_LENGTH = 0.4044  # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint", "gripper_right_finger_joint")

#

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27,
              1.32, 0.0, 1.41, 'inf', 'inf', 0.045, 0.045)

robot_parts = {}
for i, part_name in enumerate(part_names):
    robot_parts[part_name] = robot.getDevice(part_name)
    robot_parts[part_name].setPosition(float(target_pos[i]))
    robot_parts[part_name].setVelocity(
        robot_parts[part_name].getMaxVelocity() / 2.0)

# Enable gripper encoders (position sensors)
left_gripper_enc = robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc = robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)


# Enable display
display = robot.getDevice("display")
DISPLAY_DIM = 360
