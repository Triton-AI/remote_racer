#! /usr/bin python3
import rospy
from std_msgs.msg import Int32
from vesc_client import VESC_

RPM_NODE_NAME = 'vesc_rpm_node'
RPM_REQUEST_TOPIC_NAME = 'vesc_rpm_request'
RPM_ACTUAL_TOPIC_NAME = 'vesc_rpm_actual'

v = VESC_()


def callback(data):
    rpm = data.data
    v.send_rpm(rpm)
    # v.get_rpm()


if __name__ == '__main__':
    rospy.init_node(RPM_NODE_NAME, anonymous=False)
    rospy.Subscriber(RPM_REQUEST_TOPIC_NAME, Int32, callback)
    # pub = rospy.Publisher(RPM_ACTUAL_TOPIC_NAME, Int32MultiArray, queue_size=2)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()
