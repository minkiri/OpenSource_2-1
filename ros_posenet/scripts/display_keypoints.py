#!/usr/bin/env python
from __future__ import print_function
import numpy as np

import sys
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from ros_posenet.msg import Poses, Pose, Keypoint

class ImageConverter:

    def __init__(self):
        self.img = None
        self.min_part_conf = 0.1
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback_img)
        #self.keypoint_sub = rospy.Subscriber("/poses", Poses, self.callback_poses)
        #rospy.loginfo("Initialized ImageConverter")

    def callback_img(self, data):
        self.img = data
        rospy.loginfo("Received image data")
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        cv2.imshow("Image window", image_np)
        cv2.waitKey(1)

    def callback_poses(self, pose_msg):
        rospy.loginfo("Received poses data")
        if self.img is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.img, "bgr8")
            except CvBridgeError as e:
                rospy.logerr("CvBridgeError: {}".format(e))

            for i in range(len(pose_msg.poses)):
                rospy.loginfo("Processing pose {}".format(i + 1))
                keypoint_dict = {}
                for keypoint in pose_msg.poses[i].keypoints:
                    rospy.loginfo("Keypoint: {}, Score: {}, Position: ({}, {})".format(
                        keypoint.part, keypoint.score, keypoint.position.x, keypoint.position.y))
                    if keypoint.score > self.min_part_conf:
                        x, y = int(keypoint.position.x), int(keypoint.position.y)
                        keypoint_dict[keypoint.part] = (x, y)
                        cv2.circle(cv_image, (x, y), 5, (255, 0, 0), -1)

                connected_part_names = [
                    ['leftHip', 'leftShoulder'], ['leftElbow', 'leftShoulder'],
                    ['leftElbow', 'leftWrist'], ['leftHip', 'leftKnee'],
                    ['leftKnee', 'leftAnkle'], ['rightHip', 'rightShoulder'],
                    ['rightElbow', 'rightShoulder'], ['rightElbow', 'rightWrist'],
                    ['rightHip', 'rightKnee'], ['rightKnee', 'rightAnkle'],
                    ['leftShoulder', 'rightShoulder'], ['leftHip', 'rightHip']
                ]

                for connected_part1, connected_part2 in connected_part_names:
                    if all(k in keypoint_dict for k in (connected_part1, connected_part2)):
                        cv2.line(cv_image, keypoint_dict[connected_part1], keypoint_dict[connected_part2], (0, 0, 255), 3)

                if all(k in keypoint_dict for k in ('rightWrist', 'rightShoulder', 'leftWrist', 'leftShoulder')):
                    if keypoint_dict['rightWrist'][1] < keypoint_dict['rightShoulder'][1] and keypoint_dict['leftWrist'][1] < keypoint_dict['rightShoulder'][1]:
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(cv_image, 'HandsUP', (0, 20), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

                if all(k in keypoint_dict for k in ('rightHip', 'rightKnee')):
                    if keypoint_dict['rightKnee'][1] < keypoint_dict['rightHip'][1]:
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(cv_image, 'HighKneeRight', (0, 150), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

                if all(k in keypoint_dict for k in ('leftHip', 'leftKnee')):
                    if keypoint_dict['leftKnee'][1] < keypoint_dict['leftHip'][1]:
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cv2.putText(cv_image, 'HighKneeLeft', (150, 150), font, 1, (0, 0, 255), 1, cv2.LINE_AA)

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1)

def main(args):
    import pdb; pdb.set_trace()
    ic = ImageConverter()
    rospy.init_node('my_node_name', anonymous=True)
    rospy.loginfo("Starting ImageConverter node")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

