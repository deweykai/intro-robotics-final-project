from robot import robot, timestep
import bus
import math
import numpy as np

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

LIDAR_ANGLE_BINS = 667
# The sensor is so low it often hits the ground
LIDAR_SENSOR_MAX_RANGE = 8  # Meter
LIDAR_ANGLE_RANGE = math.radians(240)

lidar_offsets = np.linspace(
    +LIDAR_ANGLE_RANGE / 2.0, -LIDAR_ANGLE_RANGE / 2.0, LIDAR_ANGLE_BINS
)
# Only keep lidar readings not blocked by robot chassis
lidar_offsets = lidar_offsets[83: len(lidar_offsets) - 83]


pose_x, pose_y, pose_theta = 0, 0, 0


@bus.subscribe('/bot/pose', np.ndarray)
def gps_data(data):
    global pose_x, pose_y, pose_theta
    pose_x, pose_y, pose_theta = data


pub_lidar = bus.Publisher('/bot/sensor/lidar', list)


@bus.subscribe('/bot/cmd_tick', int)
def get_lidar_readings(_):
    readings = []
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83: len(
        lidar_sensor_readings) - 83]

    for alpha, rho in zip(lidar_offsets, lidar_sensor_readings):
        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha) * rho + 0.202
        ry = math.sin(alpha) * rho - 0.004

        # Convert detection from robot coordinates into world coordinates
        wx = math.cos(pose_theta) * rx - math.sin(pose_theta) * ry + pose_x
        wy = math.sin(pose_theta) * rx + math.cos(pose_theta) * ry + pose_y

        readings.append([wx, wy])

    pub_lidar.publish(readings)
