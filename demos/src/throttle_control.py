#!/usr/bin python3
import rospy
from std_msgs.msg import Float64

THROTTLE_NODE_NAME = "vesc_throttle_node"
THROTTLE_TOPIC_NAME = "/commands/motor/speed"


if __name__ == "__main__":
    rospy.init_node(THROTTLE_NODE_NAME, anonymous=False)
    pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float64, queue_size=1)
    RPM = Float64()
    RPM.data = 1500.0
    rate = rospy.Rate(15)
    
    while not rospy.is_shutdown():
        rate.sleep()
        pub.publish(RPM)
        rospy.spin()
