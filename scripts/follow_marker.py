#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Jungill kang 
# 2021.11.07


#######################################################################################
## ROS open manipulator custom handling node
## 
## USAGE : 
##         1. can draw circle by draw_circle service
##         2. can grab cup by grab_marker_callback function
##
##         1. can draw circle by draw_circle service
##         1. can draw circle by draw_circle service
##
##         rosrun usb_cam usb_cam_node (run usb_cam_node from usb_cam package)
##         rosrun open_manipulator_custom aruco_detect.py
##
## Service
##      /draw_circle --> draw circle
##
## Subscription
##      /rtvecs --> subscribing the rotation and translation vector
##
## Publishing
##      
## 
#######################################################################################

# scipy 0.19.1

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Empty
import os
import math
from math import cos, sin, sqrt, atan2
from scipy.optimize import fsolve
import modern_robotics as mr
from numpy import rad2deg

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                 # Uses Dynamixel SDK library
from dxlhandler import DxlHandler # custom Dynamixel handler
from open_manipulator_custom.srv import DrawCircle, DrawCircleResponse
from open_manipulator_custom.srv import SetAngle, SetAngleRequest, SetAngleResponse
from open_manipulator_custom.srv import SetAngleList, SetAngleListResponse
from open_manipulator_custom.msg import Rtvecs
from open_manipulator_custom.srv import ReadAngle, ReadAngleResponse
from open_manipulator_custom.srv import GrabMarker, GrabMarkerResponse


# dynamixel handler
dxlHandlerArray = []
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 11, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 12, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 13, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 14, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 15, DEVICENAME = '/dev/ttyUSB0'))

class followerHanlder():
    def __init__(self):
        # servcie and subscirber list
        self.marker_follower_subs = rospy.Subscriber('/rtvecs', Rtvecs, self.follower_callback, queue_size=10)
        # rotation and translation vector
        self.rvecs = np.array([0,0,0])
        self.tvecs = np.array([0,0,0])
        # id of aruco marker
        self.id = 0
        # angle of joint 1,2,3,4
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0
        # joint grab or not
        self.grab = False
        self.gripper_open = 80 # open
        self.gripper_clise = 20 # close
    
    ##############################################
    # util function
    ##############################################
    def ROTX(self, theta):
        return np.array([
            [1, 0, 0],
            [0, cos(theta), -sin(theta)],
            [0, sin(theta), cos(theta)]], dtype=float)

    def ROTY(self, theta):
        return np.array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)]], dtype=float)

    def ROTZ(self, theta):
        return np.array([
            [cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]], dtype=float)

    def angle_to_pose(self, q1, q2, q3, q4):
        BASE = [0, 0, 0]
        MOTOR2 = [0,0,77/1000]
        MOTOR3 = [(3*cos(q2)*sin(q1))/125 + (16*sin(q1)*sin(q2))/125, -(3*cos(q1)*cos(q2))/125-(16*cos(q1)*sin(q2))/125, (16*cos(q2))/125 - (3*sin(q2))/125 + 77/1000]
        MOTOR4 = [(3*cos(q2)*sin(q1))/125 + (16*sin(q1)*sin(q2))/125 - (31*sin(q1)*sin(q2)*sin(q3))/250 + (31*cos(q2)*cos(q3)*sin(q1))/250,
            (31*cos(q1)*sin(q2)*sin(q3))/250 - (16*cos(q1)*sin(q2))/125 - (31*cos(q1)*cos(q2)*cos(q3))/250 - (3*cos(q1)*cos(q2))/125,
            (16*cos(q2))/125 - (3*sin(q2))/125 - (31*cos(q2)*sin(q3))/250 - (31*cos(q3)*sin(q2))/250 + 77/1000]
        END = [((3*cos(q2)*sin(q1))/125 + (16*sin(q1)*sin(q2))/125 - (63*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/500 + (63*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/500 - (31*sin(q1)*sin(q2)*sin(q3))/250 + (31*cos(q2)*cos(q3)*sin(q1))/250),
            ((63*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/500 - (16*cos(q1)*sin(q2))/125 - (3*cos(q1)*cos(q2))/125 + (63*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/500 - (31*cos(q1)*cos(q2)*cos(q3))/250 + (31*cos(q1)*sin(q2)*sin(q3))/250),
            ((16*cos(q2))/125 - (3*sin(q2))/125 - (31*cos(q2)*sin(q3))/250 - (31*cos(q3)*sin(q2))/250 - (63*cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/500 + (63*sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/500 + 0.077)]
    
        return END

    def angle_to_rotation(self, q1, q2, q3, q4):
        R_final = np.eye(3, dtype=float)

        # world to base motor(1)
        R01 = self.ROTZ(q1)
        R_final = np.matmul(R_final, R01)

        # base motor(1) to base motor(2)
        ROTY90 = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]], dtype=float)
        R12 = np.matmul(ROTY90, self.ROTZ(q2))
        R_final = np.matmul(R_final, R12)

        # base motor(2) to base motor(3)
        R23 = self.ROTZ(q3)
        R_final = np.matmul(R_final, R23)

        # base motor(3) to base motor(4)
        R34 = self.ROTZ(q4)
        R_final = np.matmul(R_final, R34)

        # base motor(4) to end effector
        R45 = np.eye(3)
        R_final = np.matmul(R_final, R45)

        return R_final

    def pose_error(self, var, data):
        x = data[0]
        y = data[1]
        z = data[2]
        (q1,q2,q3,q4) = var
        END = [((3*cos(q2)*sin(q1))/125 + (16*sin(q1)*sin(q2))/125 - (63*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/500 + (63*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/500 - (31*sin(q1)*sin(q2)*sin(q3))/250 + (31*cos(q2)*cos(q3)*sin(q1))/250),
            ((63*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/500 - (16*cos(q1)*sin(q2))/125 - (3*cos(q1)*cos(q2))/125 + (63*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/500 - (31*cos(q1)*cos(q2)*cos(q3))/250 + (31*cos(q1)*sin(q2)*sin(q3))/250),
            ((16*cos(q2))/125 - (3*sin(q2))/125 - (31*cos(q2)*sin(q3))/250 - (31*cos(q3)*sin(q2))/250 - (63*cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/500 + (63*sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/500 + 0.077)]
    
        return [END[0]-x, END[1]-y, END[2]-z, 0]

    def rotation_error(self, var, data):
        R_desired = data[0]
        (q1,q2,q3,q4) = var

        R_current = self.angle_to_rotation(q1,q2,q3,q4)
        
        print("--R current --")
        print(R_current)
        print("--R current --")

        print("--R_desired--")
        print(R_desired)
        print("--R_desired--")

        ERROR = [
        R_desired[0][0] - R_current[0][0], R_desired[0][1] - R_current[0][1], R_desired[0][2] - R_current[0][2],
        R_desired[1][0] - R_current[1][0], R_desired[1][1] - R_current[1][1], R_desired[1][2] - R_current[1][2],
        R_desired[2][0] - R_current[2][0], R_desired[2][1] - R_current[2][1], R_desired[2][2] - R_current[2][2],]

        return ERROR

    ##############################################
    # service and publisher
    ##############################################

    # get r and t vector
    def follower_callback(self, rtvec_msg):
        id = rtvec_msg.id

        self.id = id

        print(id)

        if id == 1:
            print("recognize id ice americano")

        # get rotation and translation vector from aruco marker
        rvecs = [rtvec_msg.rvec1, rtvec_msg.rvec2, rtvec_msg.rvec3]
        tvecs = [rtvec_msg.tvec1, rtvec_msg.tvec2, rtvec_msg.tvec3]

        self.rvecs = np.array(rvecs)
        self.tvecs = np.array(tvecs)

         # get rotation and translation vector from aruco marker
        p_cam_cup = np.array(self.tvecs)

        T_base_cam = np.array([
            [0, -1 / sqrt(2), 1 / sqrt(2), 0.065],
            [-1, 0, 0, 0.115],
            [0, -1 / sqrt(2), -1 / sqrt(2), 0.265],
            [0, 0, 0, 1]]
        )

        # 1.
        # open the gripper
        # get current angle
        # rospy.wait_for_service('set_angle_service')

        # try:
        #     set_angle_req = SetAngleRequest()
        #     set_angle_req.position1 = float(self.q1)
        #     set_angle_req.position2 = float(self.q2)
        #     set_angle_req.position3 = float(self.q3)
        #     set_angle_req.position4 = float(0)
        #     set_angle_req.position5 = float(self.gripper_open)
        #     set_angle = rospy.ServiceProxy('set_angle_service', SetAngle)
        #     set_angle_resp = set_angle(set_angle_req)
        #     if set_angle_resp.success == True:
        #         print("gripper open!;")
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s" % e)

        # 2.
        # calculate the angle to the cup
        p_cam_cup = np.array([p_cam_cup[0], p_cam_cup[1], p_cam_cup[2], 1]).T
        p_base_cup = np.matmul(T_base_cam, p_cam_cup)
        rotate_angle = atan2(p_base_cup[1], p_base_cup[0])  # atan2(y,x)
        desired_z = p_cam_cup[2]

        # get current angle
        rospy.wait_for_service('read_angle_service')

        try:
            empty = Empty()
            read_angle = rospy.ServiceProxy('read_angle_service', ReadAngle)
            read_angle_resp = read_angle(empty)
            #if read_angle_resp.success == True:
                # print("read angle")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


        q0 = [read_angle_resp.position1, read_angle_resp.position2, read_angle_resp.position3,
              read_angle_resp.position4]  # read angle from the sensor

        # update angle
        self.q1 = q0[0]
        self.q2 = q0[1]
        self.q3 = q0[2]
        self.q4 = q0[3]
        
        # print(self.q1, self.q2, self.q3, self.q4)

        x = p_base_cup[0]
        y = p_base_cup[1]
        z = p_base_cup[2]

        # print("debug here")
        # print(p_cam_cup)

        # print(x,y,z)

        # q1, q2, q3 ,q4 = fsolve(self.pose_error, q0, args=[x,y,z])

        # print(q1,q2,q3,q4)
        # print(self.angle_to_pose(q1,q2,q3,q4))

        # print("debug here")
        # rospy.wait_for_service("debug")

        rospy.wait_for_service('set_angle_service')

        try:
            set_angle_req = SetAngleRequest()
            set_angle_req.position1 = float(rad2deg(rotate_angle)+90)
            set_angle_req.position2 = float(self.q2)
            set_angle_req.position3 = float(self.q3)
            set_angle_req.position4 = float(0)
            set_angle_req.position5 = float(self.gripper_open)
            set_angle = rospy.ServiceProxy('set_angle_service', SetAngle)
            set_angle_resp = set_angle(set_angle_req)
            if set_angle_resp.success == True:
                print("rotate suceed!")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # print(x, y)


def follow_marker_node():
    rospy.init_node('follower_node')
    manipulator_node = followerHanlder()
    rospy.spin()


def follow_marker_custom():
    for dxlHandler in dxlHandlerArray:
        # Open port
        dxlHandler.open_port()
        # Set port baudrate
        dxlHandler.set_baudrate()

    # start the manipulator node
    follow_marker_node()

    for dxlHandler in dxlHandlerArray:
        # Close port
        dxlHandler.close_port()


def main():
    #[x,y,z] = manipulatorHanlder.angle_to_pose(0.0, 0.0, 0.0, 0.0)
    #print(x)
    #print(y)
    #print(z)
    follow_marker_custom()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
