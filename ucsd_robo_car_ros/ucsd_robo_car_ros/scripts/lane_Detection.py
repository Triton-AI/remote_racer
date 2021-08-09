#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray, Bool
from sensor_msgs.msg import Image
from decoder import decodeImage
import time

OBSTACLE_TOPIC_NAME = '/obstacle_detection'
CAMERA_TOPIC_NAME = 'camera/color/image_raw'
CENTROID_TOPIC_NAME = '/centroid'
OBSTACLE_DETECTED_TOPIC_NAME = '/obstacle_detection'
LANE_DETECTION_NODE_NAME = 'lane_detection_node'

global obstacle_detected_result, centroid_previous
obstacle_detected_result = Bool()
centroid_previous = 0


def video_detection(data):
    global centroid_previous, obstacle_detected_result
    frame = decodeImage(data.data, data.height, data.width)
    height, width, channels = frame.shape
    # left_width = int(0)
    # right_width = width
    left_width = int(width/4)
    right_width = int(3*width/4)
    # img = cv2.cvtColor(frame[0:height, 0:width], cv2.COLOR_RGB2BGR)

    height, width, channels = frame.shape
    start_height = int(height*0.60)
    bottom_height = int(height*0.80)

    img = cv2.cvtColor(frame[start_height:bottom_height, left_width:right_width], cv2.COLOR_RGB2BGR)
    new_width = right_width - left_width

    # image pre-processing

    # experimentally found values from find_camera_values.py
    Hue_low = rospy.get_param("/Hue_low")
    Hue_high = rospy.get_param("/Hue_high")
    Saturation_low = rospy.get_param("/Saturation_low")
    Saturation_high = rospy.get_param("/Saturation_high")
    Value_low = rospy.get_param("/Value_low")
    Value_high = rospy.get_param("/Value_high")
    min_width = rospy.get_param("/Width_min")
    max_width = rospy.get_param("/Width_max")

    # get rid of white noise

    # changing color space to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # setting threshold limits for white color filter
    lower = np.array([Hue_low, Saturation_low, Value_low])
    upper = np.array([Hue_high, Saturation_high, Value_high])

    # creating mask
    mask = cv2.inRange(hsv, lower, upper)
    res_inv = cv2.bitwise_and(img, img, mask=mask)
    # res_inv = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask)) #comment when not using green filter

    # changing to gray color space
    gray = cv2.cvtColor(res_inv, cv2.COLOR_BGR2GRAY)

    # changing to black and white color space
    (dummy, blackAndWhiteImage) = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

    # locating contours in image
    contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    centers = []
    cx_list = []
    cy_list = []
    centroid_and_frame_width = []

    # plotting contours and their centroids
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if min_width < w < max_width:
            img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
            m = cv2.moments(contour)
            try:
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
                centers.append([cx, cy])
                cx_list.append(int(m['m10'] / m['m00']))
                cy_list.append(int(m['m01'] / m['m00']))
                cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

    if not obstacle_detected_result:
        try:
            if len(cx_list) >= 2:
                mid_x = int(0.5 * (cx_list[0] + cx_list[1]))
                mid_y = int(0.5 * (cy_list[0] + cy_list[1]))
                centroid_previous = mid_x
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                centroid_and_frame_width.append(mid_x)
                centroid_and_frame_width.append(new_width)
                pub.publish(data=centroid_and_frame_width)
            elif len(cx_list) == 1:
                mid_x = cx_list[0]
                mid_y = cy_list[0]
                # print(centroid_previous, mid_x)
                centroid_previous = mid_x
                cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
                centroid_and_frame_width.append(mid_x)
                centroid_and_frame_width.append(new_width)
                pub.publish(data=centroid_and_frame_width)
            elif len(cx_list) == 0:
                centroid_and_frame_width.append(-1)
                centroid_and_frame_width.append(new_width)
                pub.publish(data=centroid_and_frame_width)
        except ValueError:
            pass

    try:
        # plotting results
        # cv2.imshow("original", orig)
        cv2.imshow("mask", mask)
        cv2.imshow("gray", gray)
        cv2.imshow("res_inv", res_inv)
        # vid = cv2.flip(img, 0)
        # out.write(vid)
        cv2.imshow("blackAndWhiteImage", blackAndWhiteImage)
        cv2.imshow("plotting_centroid", img)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()


def obstacle_detected(msg):
    global obstacle_detected_result
    obstacle_detected_result = msg.data
    if obstacle_detected_result:
        rospy.Timer(rospy.Duration(3), callback)
        # pass
        # print("Pausing lane-detection node")
    else:
        pass
        # print("Resuming lane-detection node")


def callback(event):
    print("Obstacle Avoided! (Hopefully)")


if __name__ == '__main__':
    rospy.init_node(LANE_DETECTION_NODE_NAME, anonymous=False)
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,  480))
    camera_sub = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, video_detection)
    obstacle_detected_subscriber = rospy.Subscriber(OBSTACLE_DETECTED_TOPIC_NAME, Bool, obstacle_detected)
    pub = rospy.Publisher(CENTROID_TOPIC_NAME, Int32MultiArray, queue_size=2)
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
