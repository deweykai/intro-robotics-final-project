import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import robot
from robot import timestep
import math

arm_chain = Chain.from_urdf_file("robot_urdf.urdf")
print(arm_chain.links)

part_names = (
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
    "arm_7_joint",
)

for link_id in range(len(arm_chain.links)):

    # This is the actual link object
    link = arm_chain.links[link_id]

    # I've disabled "torso_lift_joint" manually as it can cause
    # the TIAGO to become unstable.
    if link.name not in part_names or link.name == "torso_lift_joint":
        print("Disabling {}".format(link.name))
        arm_chain.active_links_mask[link_id] = False

motors = []
for link in arm_chain.links:
    if link.name in part_names and link.name != "torso_lift_joint":
        motor = robot.getDevice(link.name)

        # Make sure to account for any motors that
        # require a different maximum velocity!
        if link.name == "torso_lift_joint":
            motor.setVelocity(0.07)
        else:
            motor.setVelocity(1)

        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timestep)
        motors.append(motor)

prev_target = None
# get from vision.py
target = None

offset_target = [-(target[2]) + 0.22, -target[0] + 0.08, (target[1]) + 0.97 + 0.2]
initial_position = (
    [0, 0, 0, 0] + [m.getPositionSensor().getValue() for m in motors] + [0, 0, 0, 0]
)

ikResults = arm_chain.inverse_kinematics(
    offset_target,
    initial_position=initial_position,
    target_orientation=[0, 0, 1],
    orientation_mode="Y",
)

error = 0
for item in range(3):
    error += (offset_target[item] - prev_target[item]) ** 2
error = math.sqrt(error)

for res in range(len(ikResults)):
    if arm_chain.links[res].name in part_names:
        robot.getDevice(arm_chain.links[res].name).setPosition(ikResults[res])
        print("Setting {} to {}".format(arm_chain.links[res].name, ikResults[res]))

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

ax = matplotlib.pyplot.figure().add_subplot(111, projection="3d")

arm_chain.plot(initial_position, ax, target=prev_target)

arm_chain.plot(ikResults, ax, target=prev_target)
matplotlib.pyplot.show()

prev_target = offset_target
