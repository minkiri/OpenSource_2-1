#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

def process_image(image):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(image, "bgr8")

    color_index = 0

    if frame is not None:
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        lower_blue = np.array([90, 100, 100])
        upper_blue = np.array([130, 255, 255])

        img_red = cv2.inRange(frame_hsv, lower_red, upper_red)
        img_green = cv2.inRange(frame_hsv, lower_green, upper_green)
        img_blue = cv2.inRange(frame_hsv, lower_blue, upper_blue)

        cv2.imshow("img_red", img_red)
        cv2.imshow("img_green", img_green)
        cv2.imshow("img_blue", img_blue)

        cv2.waitKey(1)

        kernel = np.ones((5, 5), np.uint8)
        img_red = cv2.morphologyEx(img_red, cv2.MORPH_OPEN, kernel)
        img_green = cv2.morphologyEx(img_green, cv2.MORPH_OPEN, kernel)
        img_blue = cv2.morphologyEx(img_blue, cv2.MORPH_OPEN, kernel)

        if np.sum(img_red) > 0:
            color_index = 1
            contours, _ = cv2.findContours(img_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, (0, 0, 255), 2)
            height, width, _ = frame.shape
            bbox_percentage = (cv2.contourArea(contours[0]) / (height * width)) * 100

            if bbox_percentage >= 2.0:
                color_index = 1
            else:
                color_index = 0

        elif np.sum(img_green) > 0:
            color_index = 2
            contours, _ = cv2.findContours(img_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            height, width, _ = frame.shape
            bbox_percentage = (cv2.contourArea(contours[0]) / (height * width)) * 100

            if bbox_percentage >= 2.0:
                color_index = 2
            else:
                color_index = 0

        elif np.sum(img_blue) > 0:
            color_index = 3
            contours, _ = cv2.findContours(img_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, (255, 0, 0), 2)
            height, width, _ = frame.shape
            bbox_percentage = (cv2.contourArea(contours[0]) / (height * width)) * 100

            if bbox_percentage >= 2.0:
                color_index = 3
            else:
                color_index = 0

        else:
            color_index = 0

        rospy.loginfo("감지된 색인덱스: %d", color_index)
        pub.publish(color_index)

        cv2.imshow("frame", frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('color_detection_node')

    pub = rospy.Publisher('color_image', Int32, queue_size=10)
    rospy.Subscriber('/usb_cam/image_rect_color', Image, process_image)

    rospy.spin()
