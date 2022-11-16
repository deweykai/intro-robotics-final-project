"""Vision Processing

This module take vision sensor data and processes into a more usable format

This file provides:
    * lidar_hits - list of (x,y) coordinates where lidar

This module depends on:
    * localization - for caluclating the position of lidar_hits
"""


import numpy as np
import localization as loc
import math
from robot import lidar, camera
import numpy as np
import copy
import cv2

LIDAR_ANGLE_BINS = 667
# The sensor is so low it often hits the ground
LIDAR_SENSOR_MAX_RANGE = 2  # Meter
LIDAR_ANGLE_RANGE = math.radians(240)

lidar_offsets = np.linspace(
    +LIDAR_ANGLE_RANGE / 2.0, -LIDAR_ANGLE_RANGE / 2.0, LIDAR_ANGLE_BINS
)
# Only keep lidar readings not blocked by robot chassis
lidar_offsets = lidar_offsets[83: len(lidar_offsets) - 83]


def get_lidar_readings():
    readings = []
    pose_x, pose_y, pose_theta = loc.pose_x, loc.pose_y, loc.pose_theta
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

    return readings


color_ranges = []


def add_color_range_to_detect(lower_bound, upper_bound):
    """
    @param lower_bound: Tuple of BGR values
    @param upper_bound: Tuple of BGR values
    """
    global color_ranges
    color_ranges.append([lower_bound, upper_bound])


def check_if_color_in_range(bgr_tuple):
    """
    @param bgr_tuple: Tuple of BGR values
    @returns Boolean: True if bgr_tuple is in any of the color ranges specified in color_ranges
    """
    for entry in color_ranges:
        lower, upper = entry[0], entry[1]
        in_range = True
        for i in range(len(bgr_tuple)):
            if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
                in_range = False
                break
        if in_range:
            return True
    return False


def do_color_filtering(img):
    img_height = img.shape[0]
    img_width = img.shape[1]

    mask = np.zeros([img_height, img_width])

    for y in range(img_height):
        for x in range(img_width):
            if check_if_color_in_range(img[y, x]):
                mask[y, x] = 1

    return mask


def expand_nr(img_mask, cur_coord, coordinates_in_blob):
    coordinates_in_blob = []
    coordinate_list = [cur_coord]
    while len(coordinate_list) > 0:
        cur_coordinate = coordinate_list.pop()
        if (
            cur_coordinate[0] < 0
            or cur_coordinate[1] < 0
            or cur_coordinate[0] >= img_mask.shape[0]
            or cur_coordinate[1] >= img_mask.shape[1]
        ):
            continue
        if img_mask[cur_coordinate[0], cur_coordinate[1]] == 0.0:
            continue
        img_mask[cur_coordinate[0], cur_coordinate[1]] = 0
        coordinates_in_blob.append(cur_coordinate)
        above = [cur_coordinate[0] - 1, cur_coordinate[1]]
        below = [cur_coordinate[0] + 1, cur_coordinate[1]]
        left = [cur_coordinate[0], cur_coordinate[1] - 1]
        right = [cur_coordinate[0], cur_coordinate[1] + 1]
        for coord in [above, below, left, right]:
            coordinate_list.append(coord)

    return coordinates_in_blob


def get_blobs(img_mask):
    img_mask_height = img_mask.shape[0]
    img_mask_width = img_mask.shape[1]

    temp_img_mask = copy.copy(img_mask)
    blobs_list = []

    for y in range(img_mask_height):
        for x in range(img_mask_width):
            if img_mask[y, x] == 1:
                blob_coords = expand_nr(temp_img_mask, (y, x), [])
                blobs_list.append(blob_coords)

    return blobs_list


def get_blob_centroids(blobs_list):
    object_positions_list = []

    for list in blobs_list:
        if len(list) < 120:
            continue
        object_positions_list.append(np.mean(list, axis=0))
    return object_positions_list


def init():
    global img
    img = []
    """Initialize vision module"""
    add_color_range_to_detect([0, 0, 210], [80, 92, 255])
    add_color_range_to_detect([0, 180, 0], [146, 255, 131])
    add_color_range_to_detect([200, 0, 0], [255, 119, 47])
    add_color_range_to_detect([0, 210, 230], [63, 250, 255])

    image = camera.getImageArray()
    if image:
        for y in range(0, camera.getHeight()):
            temp_list = []
            for x in range(0, camera.getWidth()):
                red = image[x][y][0]
                green = image[x][y][1]
                blue = image[x][y][2]
                temp_list.append([blue, green, red])
            img.append(temp_list)
    img = np.asarray(img)
    img = img.astype(np.uint8)
    img_mask = do_color_filtering(img)
    blobs = get_blobs(img_mask)

    object_positions_list = get_blob_centroids(blobs)

    img_markup = img.copy()
    for obj_pos in object_positions_list:
        obj_pos_vector = np.array(obj_pos).astype(np.int32)
        img_markup = cv2.circle(
            img_markup, (obj_pos_vector[1],
                         obj_pos_vector[0]), 5, (0, 0, 0), 10
        )
        print("Object pos: " + str(obj_pos_vector))

    # causes controller to freeze
    # cv2.imshow("orig", img)
    # cv2.imshow("mask", img_mask)
    # cv2.imshow("located", img_markup)
    # cv2.waitKey(-1)
    # cv2.destroyAllWindows()


def update():
    """Update hook for vision module"""
    # TODO: process lider readings into lider_hits
    pass
