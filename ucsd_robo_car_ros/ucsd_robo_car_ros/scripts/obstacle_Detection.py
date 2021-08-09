#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import LaserScan
import math
import time

global normalized_angle, min_distance_data

LASER_SCAN_TOPIC_NAME = '/scan'
OBSTACLE_ANGLE_TOPIC_NAME = '/obstacle_angle'
OBSTACLE_DETECTED_TOPIC_NAME = '/obstacle_detection'
OBSTACLE_DETECTION_NODE_NAME = 'obstacle_detection_node'


def callback(data):
    global normalized_angle, min_distance_data
    # full_right = data.data[0]
    # straight = data.data[1]
    # full_left = data.data[2]
    # laser_data = data.data
    # starting_angle = -45
    # end_angle = 225
    degree_off_set = 45

    max_distance_tolerance = 0.6
    min_distance_tolerance = 0.2
    normalized_angle = Float32()
    min_distance_data = Float32()
    # laser_data = data.ranges[134:676]  # slice to get values from 0-180deg
    laser_data = data.ranges[269:539]  # slice to get values from 45-135deg
    min_distance_data = min(laser_data)
    # print(min_distance_data)

    total_number_of_scans = len(laser_data)
    scans_per_degree = 3
    # viewing_angle = total_number_of_scans / scans_per_degree

    # print(min_distance_data)
    # min_distance_angle = laser_data.index(min(laser_data)) + degree_off_set
    # angle_degrees = min_distance_angle / scans_per_degree
    # print(angle_degrees, min_distance_data)

    # values at 45 degree
    angle_45 = scans_per_degree * 90
    angle_45_range = data.ranges[angle_45]

    # values at 67.5 degree
    angle_67 = scans_per_degree * 112
    angle_67_range = data.ranges[angle_67]

    # values at 90 degree
    angle_90 = scans_per_degree * 135
    angle_90_range = data.ranges[angle_90]

    # values at 112.5 degree
    angle_112 = scans_per_degree * 157
    angle_112_range = data.ranges[angle_112]

    # values at 135 degree
    angle_135 = scans_per_degree * 180
    angle_135_range = data.ranges[angle_135]

    range_values = [angle_45_range, angle_67_range, angle_90_range, angle_112_range, angle_135_range]
    angle_values = [45, 67.5, 90, 112.5, 135]
    min_distance = min(range_values)
    min_angle_index = range_values.index(min(range_values))
    min_angle = angle_values[min_angle_index]
    # print(min_angle, min_distance)
    obstacle_detected = Bool()

    if max_distance_tolerance >= abs(min_distance) >= min_distance_tolerance:
        angle_rad = (min_angle * math.pi) / 180
        normalized_angle = round(math.cos(angle_rad))
        obstacle_detected.data = True
        obstacle_pub.publish(obstacle_detected)
        angle_pub.publish(normalized_angle)
        dist_pub.publish(min_distance_data)

    else:
        obstacle_detected.data = False
        obstacle_pub.publish(obstacle_detected)


    # if max_distance_tolerance >= abs(min_distance_data) >= min_distance_tolerance:
    #     min_distance_angle = laser_data.index(min(laser_data)) + degree_off_set
    #     angle_degrees = min_distance_angle / scans_per_degree
    #     angle_rad = (angle_degrees * math.pi) / 180
    #     normalized_angle = math.cos(angle_rad)
    #     angle_pub.publish(normalized_angle)
    #     dist_pub.publish(min_distance_data)

        # full_left_angle = scans_per_degree * 180
        # full_left_range = msg.ranges[full_left_angle]

    # if full_right <= min_distance_tolerance:
    #     normalized_angle = 1.0
    # elif straight <= min_distance_tolerance:
    #     normalized_angle = 0.0
    # elif full_left <= min_distance_tolerance:
    #     normalized_angle = -1.0
    # pub.publish(normalized_angle)
    # time.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node(OBSTACLE_DETECTION_NODE_NAME, anonymous=False)
    sub = rospy.Subscriber(LASER_SCAN_TOPIC_NAME, LaserScan, callback)
    angle_pub = rospy.Publisher(OBSTACLE_ANGLE_TOPIC_NAME, Float32, queue_size=1)
    obstacle_pub = rospy.Publisher(OBSTACLE_DETECTED_TOPIC_NAME, Bool, queue_size=1)
    dist_pub = rospy.Publisher('distance_test', Float32, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
