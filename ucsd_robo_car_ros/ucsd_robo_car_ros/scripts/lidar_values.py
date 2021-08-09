#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

'''
lidar specs: https://www.sick.com/de/en/detection-and-ranging-solutions/2d-lidar-sensors/tim5xx/tim571-2050101/p/p412444
Lidar working range
min angle: -45deg or from 90deg heading, 135deg or 3*pi/4
max angle: 225deg or from 90deg heading, -135deg or -3*pi/4
viewing angle: 270deg or 3*pi/2
total number of scans: 811
scans_per_degree = 811/270 = 3.003 ~= 3

0deg (full right) --> scans_per_degree * 45
90deg (straight forward) --> scans_per_degree * 135
180deg (full left) --> scans_per_degree * 225
full_right
full_left
straight_forward
'''


def callback(msg):
    scan_simple_array = []
    viewing_angle = 270
    total_number_of_scans = len(msg.ranges)
    scans_per_degree = int(total_number_of_scans/viewing_angle)

    # values at 45 degree
    full_right_angle = scans_per_degree * 90
    full_right_range = msg.ranges[full_right_angle]

    # values at 90 degree
    straight_forward_angle = scans_per_degree * 135
    straight_forward_range = msg.ranges[straight_forward_angle]

    # values at 135 degree
    full_left_angle = scans_per_degree * 180
    full_left_range = msg.ranges[full_left_angle]

    # print msg.ranges[full_right]
    # print msg.ranges[straight_forward]
    # print msg.ranges[full_left]

    scan_simple_array.append(full_right_range)
    scan_simple_array.append(straight_forward_range)
    scan_simple_array.append(full_left_range)

    my_array_for_publishing = Float32MultiArray(data=scan_simple_array)
    pub.publish(my_array_for_publishing)


if __name__ == '__main__':
    rospy.init_node('scan_values', anonymous=False)
    rate = rospy.Rate(15)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher("/scan_values_simple", Float32MultiArray, queue_size=3)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
