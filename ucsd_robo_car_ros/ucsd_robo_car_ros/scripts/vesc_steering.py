#!/usr/bin python3
import rospy
from std_msgs.msg import Float32
from vesc_client import VESC_

STEERING_NODE_NAME = 'vesc_steering_node'
STEERING_TOPIC_NAME = 'vesc_steering'

v = VESC_()


def callback(data):
    vesc_min_limit = 0
    vesc_max_limit = 1
    data_min_limit = -1
    data_max_limit = 1

    steering_angle = float(-0.1 + ((data.data-data_min_limit)*(vesc_max_limit - vesc_min_limit))/(data_max_limit-data_min_limit))  # mapping from [-1,1] --> [0,1]
    v.send_servo_angle(steering_angle)


if __name__ == '__main__':
    rospy.init_node(STEERING_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callback)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()
