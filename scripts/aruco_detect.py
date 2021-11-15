#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Jungill kang 
# 2021.11.11

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from open_manipulator_custom.msg import Rtvecs

# arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
# arucoParams = cv2.aruco.DetectorParameters_create()
# (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
# 	parameters=arucoParams)

# calibrated camera coefficietns
camMatrix = np.array([[637.51624, 0., 317.66075],
                                [0.,638.20674, 225.52934],
                                [0., 0., 1.]])
distCoeffs = np.array([0.063204, -0.292048, 0.001874, 0.002631, 0.000000]).reshape(5,1)

marker_length = 0.1 # m

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters_create()

class camHandler():
    def __init__(self):
        self.selecting_sub_image = "raw" # you can choose image type "compressed", "raw"
 
        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback, queue_size=1)
 
        self.rtpub = rospy.Publisher('/rtvecs', Rtvecs, queue_size=10)
        self.bridge = CvBridge()
        # self.timer = rospy.Rate(100) # 100Hz
 
    def callback(self, image_msg):
        if self.selecting_sub_image == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.selecting_sub_image == "raw":
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
 
        cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_gray, arucoDict,
            parameters=arucoParams)

         # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        fixedCorners = []

        for corner in corners:
            corner2 = cv2.cornerSubPix(cv_gray, corner, winSize=(3, 3), zeroZone=(-1, -1), criteria=criteria)
            fixedCorners.append(corner2)
        
        # verify *at least* one ArUco marker was detected
        if np.all(ids is not None):        
            rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners=fixedCorners, markerLength=marker_length, cameraMatrix=camMatrix, distCoeffs=distCoeffs)
            #(rvecs-tvecs).any() 
            cv2.aruco.drawDetectedMarkers(cv_image, corners)
            cv2.aruco.drawAxis(cv_image,camMatrix,distCoeffs,rvecs,tvecs,0.1)
            print(rvecs.reshape(-1,3))
            print(tvecs.reshape(-1,3))
            
            rvecmsg = rvecs.reshape(-1,3)
            tvecmsg = tvecs.reshape(-1,3)
            for i in range(len(ids)):
                msg = Rtvecs()
                msg.id = ids[i]
                msg.rvec1 = rvecmsg[i][0]
                msg.rvec2 = rvecmsg[i][1]
                msg.rvec3 = tvecmsg[i][2]
                msg.tvec1 = tvecmsg[i][0]
                msg.tvec2 = tvecmsg[i][1]
                msg.tvec3 = tvecmsg[i][2]
                self.rtpub.publish(msg)


        cv2.imshow('cv_image', cv_image), cv2.waitKey(1)

def aruco_detect_node():
    rospy.init_node('aruco_detect_node')
    aruco_detect_node = camHandler()
    rospy.spin()
    cv2.destroyAllWindows()


def main():
    aruco_detect_node()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
