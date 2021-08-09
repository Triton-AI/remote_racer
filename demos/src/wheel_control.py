#!/usr/bin python3
import random
import rospy
from std_msgs.msg import Float64

STEERING_NODE_NAME = "vesc_steering_node"
STEERING_TOPIC_NAME = "/commands/servo/position"


if __name__ == "__main__":
    rospy.init_node(STEERING_NODE_NAME, anonymous=False)
    pub = rospy.Publisher(STEERING_TOPIC_NAME, Float64, queue_size=1)

    val = Float64()

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()
        val.data = round(random.uniform(-0.4, 0.7), 2)
        pub.publish(val)
        rospy.spin()

