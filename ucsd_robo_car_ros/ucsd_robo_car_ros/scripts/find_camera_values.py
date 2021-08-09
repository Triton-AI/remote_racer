#!/usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(-1)


def callback(x):
    pass


cv2.namedWindow('sliders')

lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255
min_width = 10
max_width = 500

cv2.createTrackbar('lowH', 'sliders', lowH, highH, callback)
cv2.createTrackbar('highH', 'sliders', lowH, highH, callback)

cv2.createTrackbar('lowS', 'sliders', lowS, highS, callback)
cv2.createTrackbar('highS', 'sliders', highS, highS, callback)

cv2.createTrackbar('lowV', 'sliders', lowV, highV, callback)
cv2.createTrackbar('highV', 'sliders', highV, highV, callback)

cv2.createTrackbar('min_width', 'sliders', min_width, max_width, callback)
cv2.createTrackbar('max_width', 'sliders', min_width, max_width, callback)


def camera_values():
    ret, img = cap.read()
    height, width, channels = img.shape
    start_height = int(height * 0.50)
    bottom_height = int(height * 0.80)

    left_width = int(0)
    right_width = int(width)

    # img = cv2.cvtColor(img[start_height:bottom_height, left_width:right_width], cv2.COLOR_RGB2BGR)
    img = img[start_height:bottom_height, left_width:right_width]

    # get trackbar positions
    lowH = cv2.getTrackbarPos('lowH', 'sliders')
    highH = cv2.getTrackbarPos('highH', 'sliders')
    lowS = cv2.getTrackbarPos('lowS', 'sliders')
    highS = cv2.getTrackbarPos('highS', 'sliders')
    lowV = cv2.getTrackbarPos('lowV', 'sliders')
    highV = cv2.getTrackbarPos('highV', 'sliders')
    min_width = cv2.getTrackbarPos('min_width', 'sliders')
    max_width = cv2.getTrackbarPos('max_width', 'sliders')

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # for webcam
    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # for intel
    lower = np.array([lowH, lowS, lowV])
    higher = np.array([highH, highS, highV])
    mask = cv2.inRange(hsv, lower, higher)
    res = cv2.bitwise_and(img, img, mask=mask)

    # changing to gray color space
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    # changing to black and white color space
    gray_lower = 127
    gray_upper = 255
    (dummy, blackAndWhiteImage) = cv2.threshold(gray, gray_lower, gray_upper, cv2.THRESH_BINARY)
    contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    centers = []
    cx_list = []
    cy_list = []
    centroid_and_frame_width = []

    # plotting contours and their centroids
    for contour in contours:
        area = cv2.contourArea(contour)
        x, y, w, h = cv2.boundingRect(contour)
        if min_width < w < max_width:
            try:
                img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
                m = cv2.moments(contour)
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
                centers.append([cx, cy])
                cx_list.append(int(m['m10'] / m['m00']))
                cy_list.append(int(m['m01'] / m['m00']))
                cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

    try:
        if len(cx_list) >= 2:
            mid_x = int(0.5 * (cx_list[0] + cx_list[1]))
            mid_y = int(0.5 * (cy_list[0] + cy_list[1]))
            cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
        elif len(cx_list) == 1:
            mid_x = cx_list[0]
            mid_y = cy_list[0]
            cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
    except ValueError:
        pass

    # plotting results
    try:
        cv2.imshow('img', img)
        cv2.imshow('mask', mask)
        cv2.imshow('res', res)
        cv2.imshow('gray', gray)
        cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    while True:
        camera_values()