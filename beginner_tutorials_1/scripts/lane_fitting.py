#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os, rospkg
import json

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

from utils import BEVTransform, CURVEFit, draw_lane_img



class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.callback)
        self.img_bgr = None
        self.img_lane = None
        self.edges = None

        self.lower_wlane = np.array([8,8,150])
        self.upper_wlane = np.array([180,50,255])           

        self.lower_ylane = np.array([10,100,100])
        self.upper_ylane = np.array([30,255,255])

        self.crop_pts = np.array([[[0,400],
                [80,275],
                [160,150],
                [460,150],
                [550,275],[640,400]]])
        self.mtx = np.array([[[840.1893, 0, 682.598],
                            [0,840.0267,488.9279],
                            [0, 0, 1]]])
        self.dist = np.array([[[-0.3354, 0.0943]]])

        

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            # h,  w = img.shape[:2]
            # newcameramtx, roi=cv2.getOptimalNewCameraMatrix(self.mtx ,self.dist,(w,h),1,(w,h))
            # mapx,mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,newcameramtx,(w,h),5)
            # self.img_dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)
            

        except CvBridgeError as e:
            print(e)

    def binarize(self, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        img_canny= cv2.Canny(img_gray, 130, 200)
        img_sobel = cv2.Sobel(img_canny, -1, 1, 0)
        kernel = np.ones((3,3),np.uint8)
        img_sobel = cv2.dilate(img_sobel,kernel,iterations=5)

        # cv2.imshow("canny",img_canny)
        # cv2.imshow("sobel", img_sobel)
        # img_canny_y = cv2.Canny(img_gray, 150, 200)

        
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_wlane = cv2.inRange(img_hsv, self.lower_wlane, self.upper_wlane)
        img_ylane = cv2.inRange(img_hsv, self.lower_ylane, self.upper_ylane)
        
        img_verify_w = cv2.bitwise_and(img_sobel, img_wlane)
        img_verify_y = cv2.bitwise_and(img_sobel, img_ylane)
        
        self.img_lane = cv2.bitwise_or(img_verify_w, img_verify_y)
        #self.img_lane = cv2.bitwise_or(img_wlane, img_ylane)
        
        kernel = np.ones((5,5),np.uint8)
        # img_open = cv2.morphologyEx(self.img_lane,cv2.MORPH_OPEN,kernel)
        img_erode = cv2.dilate(self.img_lane,kernel,iterations=5)
        # cv2.imshow("img_erode",img_erode)
        # self.img_lane = img_open
        


        #cv2.imshow("img",img)
        # cv2.imshow("img_wlane",img_wlane)
        # cv2.imshow("img_ylane",img_ylane)
        # cv2.imshow("img_verify_w",img_verify_w)
        # cv2.imshow("img_verify_y",img_verify_y)
        #cv2.imshow("img_lane",self.img_lane)
        # cv2.waitKey(1)
        # self.img_lane = cv2.bitwise_or(img_wlane, img_ylane)
        return self.img_lane

    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape)==3:

            # num of channel = 3

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        else:
    
            # grayscale

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask = cv2.bitwise_and(mask, img)
        # cv2.imshow("ROI",mask)
        # cv2.waitKey(0)
        return mask


if __name__ == '__main__':
    
    rp = rospkg.RosPack()
    
    currentPath = rp.get_path("beginner_tutorials")
    
    with open(os.path.join(currentPath, 'sensor/sensor_params.json'), 'r') as fp:
        sensor_params = json.load(fp)

    params_cam = sensor_params["params_cam"]

    rospy.init_node('lane_fitting', anonymous=True)

    image_parser = IMGParser()
    bev_op = BEVTransform(params_cam=params_cam)
    curve_learner = CURVEFit(order=3, lane_width=5 ,y_margin=1, x_range=30, min_pts=50)
    #curve_learner = CURVEFit(order=3, lane_width=5 ,y_margin=1, x_range=30, min_pts=50, loss='absolute_error')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if image_parser.img_bgr  is not None:

            img_crop = image_parser.mask_roi(image_parser.img_bgr)

            img_warp = bev_op.warp_bev_img(img_crop)

            img_lane = image_parser.binarize(img_warp)

            img_f = bev_op.warp_inv_img(img_lane)

            lane_pts = bev_op.recon_lane_pts(img_f)

            x_pred, y_pred_l, y_pred_r = curve_learner.fit_curve(lane_pts)

            curve_learner.write_path_msg(x_pred, y_pred_l, y_pred_r)

            curve_learner.pub_path_msg()

            xyl, xyr = bev_op.project_lane2img(x_pred, y_pred_l, y_pred_r)

            img_lane_fit = draw_lane_img(img_lane, xyl[:, 0].astype(np.int32),
                                                xyl[:, 1].astype(np.int32),
                                                xyr[:, 0].astype(np.int32),
                                                xyr[:, 1].astype(np.int32))

            cv2.imshow("birdview", img_lane_fit)
            # cv2.imshow("img_warp", img_warp)
            # cv2.imshow("origin_img", image_parser.img_bgr)

            cv2.waitKey(1)

            rate.sleep()
