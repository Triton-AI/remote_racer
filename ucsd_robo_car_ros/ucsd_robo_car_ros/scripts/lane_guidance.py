#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import Float32, Int32, Int32MultiArray

STEERING_TOPIC_NAME = '/vesc_steering'
RPM_REQUEST_TOPIC_NAME = 'vesc_rpm_request'
CENTROID_TOPIC_NAME = '/centroid'
OBSTACLE_ANGLE_TOPIC_NAME = '/obstacle_angle'
OBSTACLE_DETECTED_TOPIC_NAME = '/obstacle_detection'
LANE_GUIDANCE_NODE_NAME = 'lane_guidance_node'

global steering_float, rpm_int
steering_float = Float32()
rpm_int = Int32()

error_dic = {'previous_error': 0.0, 'normalized_error': 0.0}
time_dic = {'previous_time': 0.0, 'current_time': 0.0}


def line_follower(data):
    global steering_float, rpm_int
    steering_float = Float32()
    rpm_int = Int32()
    centroid = data.data[0]
    width = data.data[1]  # width of camera frame

    if centroid == 0:
        steering_float = 0.0
        steering_pub.publish(steering_float)
    elif centroid == -1:
        normalized_error = None
    else:
        error_x = float(centroid - (width / 2))
        normalized_error = float(error_x / (width / 2))

        t = rospy.Time.from_sec(time.time())
        current_time = t.to_sec()  # floating point
        time_dic['current_time'] = current_time
        previous_time = time_dic.get('previous_time')
        delta_time = current_time - previous_time
        previous_error = error_dic.get('previous_error')
        de_dt = (normalized_error - previous_error) / delta_time
        integral_error = 0
        integral_error = integral_error + normalized_error * delta_time

        kp = 0.4
        kd = 0.000001
        ki = 0.000000

        print("previous error: ", previous_error)
        print("previous error: ", error_x)
        print("previous time: ", time_dic['previous_time'])
        print("current time: ", time_dic['current_time'])
        time_dic['previous_time'] = current_time
        error_dic['previous_error'] = error_x

    rpm_int = 1500
    if normalized_error is None:
        pass
    else:
        control_signal = kp * normalized_error + kd * de_dt + ki * integral_error
        print("control_signal: ", control_signal)
        steering_float = float(control_signal)
        if steering_float < -1:
            steering_float = -1
        elif steering_float > 1:
            steering_float = 1
        elif steering_float < 0:
            steering_float = steering_float*1.4
        elif steering_float > 0:
            steering_float = steering_float*1.4
        else:
            pass
        steering_pub.publish(steering_float)
    throttle_pub.publish(rpm_int)


def obstacle_avoid(data):
    global steering_float, rpm_int
    rpm_int = 1500
    steering_float = -1 * data.data
    if steering_float == 0.0:
        steering_float = -1.0
    rospy.Timer(rospy.Duration(3), callback)
    steering_pub.publish(steering_float)
    throttle_pub.publish(rpm_int)


def callback(event):
    # global steering_float
    # steering_float = 0.0
    # steering_pub.publish(steering_float)
    print("Obstacle Avoided! (Hopefully)")


if __name__ == '__main__':
    rospy.init_node(LANE_GUIDANCE_NODE_NAME, anonymous=False)
    centroid_subscriber = rospy.Subscriber(CENTROID_TOPIC_NAME, Int32MultiArray, line_follower)
    obstacle_angle_subscriber = rospy.Subscriber(OBSTACLE_ANGLE_TOPIC_NAME, Float32, obstacle_avoid)
    steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
    throttle_pub = rospy.Publisher(RPM_REQUEST_TOPIC_NAME, Int32, queue_size=1)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
