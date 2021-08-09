#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

STEERING_NODE_NAME = 'steering_client'
STEERING_TOPIC_NAME = 'steering'

'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 1 
    fully right: -1
    fully left : 1 
'''

kit = ServoKit(channels=16)


def callback(data):
    normalized_steering = data.data
    angle_delta = normalized_steering * 90  # difference in degrees from the center 90 degrees
    kit.servo[15].angle = 90 + angle_delta


# def listener():
#     rospy.init_node(STEERING_NODE_NAME, anonymous=False)
#     rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callback)
#     rospy.spin()
#
#
# if __name__ == '__main__':
#     listener()

if __name__ == '__main__':
    rospy.init_node(STEERING_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callback)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()


