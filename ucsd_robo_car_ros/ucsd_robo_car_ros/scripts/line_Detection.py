#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image
from decoder import decodeImage

OBSTACLE_TOPIC_NAME = '/obstacle_detection'
CAMERA_TOPIC_NAME = 'camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'
OBSTACLE_DETECTED_TOPIC_NAME = '/obstacle_detection'
LANE_DETECTION_NODE_NAME = 'lane_detection_node'


def video_detection(data):
    # decode image
    frame = decodeImage(data.data, data.height, data.width)
    height, width, channels = frame.shape
    img = cv2.cvtColor(frame[0:height, 0:width], cv2.COLOR_RGB2BGR)


    # getting and setting image properties
    rows_to_watch = 50
    rows_offset = 150
    top_height = height - rows_offset
    bottom_height = top_height + rows_to_watch

    # img = frame[top_height:bottom_height, 0:width]
    orig = img.copy()

    # experimentally found values from find_camera_values.py
    Hue_low = 0
    Hue_high = 77
    Saturation_low = 77
    Saturation_high = 255
    Value_low = 195
    Value_high = 255

    # changing color space to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # setting threshold limits for yellow color filter
    lower = np.array([Hue_low, Saturation_low, Value_low])
    upper = np.array([Hue_high, Saturation_high, Value_high])

    # creating mask
    mask = cv2.inRange(hsv, lower, upper)

    m = cv2.moments(mask, False)
    try:
        cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
    except ZeroDivisionError:
        cy, cx = int(height / 2), int(width / 2)

    # Publish centroid
    centroid_and_frame_width = []
    mid_x.data = cx
    mid_y.data = cy
    cv2.circle(img, (mid_x.data, mid_y.data), 7, (255, 0, 0), -1)
    centroid_and_frame_width.append(mid_x.data)
    centroid_and_frame_width.append(width)
    pub.publish(data=centroid_and_frame_width)

    # plotting results
    try:
        cv2.imshow("original", orig)
        cv2.imshow("yellow mask", mask)
        cv2.imshow("plotting centroid", img)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('lane_detection_node', anonymous=True)
    camera_sub = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, video_detection)
    pub = rospy.Publisher(CENTROID_TOPIC_NAME, Int32MultiArray, queue_size=2)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()
        rospy.spin()
