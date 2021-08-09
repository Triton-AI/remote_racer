#! /usr/bin/env python

from tf import TransformBroadcaster
import rospy
from rospy import Time
from nav_msgs.msg import Odometry


def callback(msg):
    b = TransformBroadcaster()
    x = msg.pose.pose.x
    y = msg.pose.pose.x

    rotation = (0.0, 0.0, 0.0, 1.0)

    translation = (x, y, 0.0)

    b.sendTransform(translation, rotation, Time.now(), 'base_link', '/map')


if __name__ == '__main__':
    rospy.init_node("my_broadcaster", anonymous=False)
    rospy.Subscriber("/vesc/odom", Odometry, callback)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()