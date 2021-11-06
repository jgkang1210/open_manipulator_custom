#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Jungill kang 
# 2021.11.07

# scipy 0.19.1

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
import os
import math
from math import cos, sin
from scipy.optimize import fsolve

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
from open_manipulator_custom.srv import SetAngleList, SetAngleListResponse


# dynamixel handler
dxlHandlerArray = []
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 11, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 12, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 13, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 14, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 15, DEVICENAME = '/dev/ttyUSB0'))

def angle_to_pose(q1,q2,q3,q4):
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

def pose_error(var, data):
    x = data[0]
    y = data[1]
    z = data[2]
    (q1,q2,q3,q4) = var
    END = [((3*cos(q2)*sin(q1))/125 + (16*sin(q1)*sin(q2))/125 - (63*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/500 + (63*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/500 - (31*sin(q1)*sin(q2)*sin(q3))/250 + (31*cos(q2)*cos(q3)*sin(q1))/250),
        ((63*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/500 - (16*cos(q1)*sin(q2))/125 - (3*cos(q1)*cos(q2))/125 + (63*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/500 - (31*cos(q1)*cos(q2)*cos(q3))/250 + (31*cos(q1)*sin(q2)*sin(q3))/250),
        ((16*cos(q2))/125 - (3*sin(q2))/125 - (31*cos(q2)*sin(q3))/250 - (31*cos(q3)*sin(q2))/250 - (63*cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/500 + (63*sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/500 + 0.077)]
 
    return [END[0]-x, END[1]-y, END[2]-z, 0]

def draw_circle_callback(req):
    rsp = DrawCircleResponse()
    
    offset = [0.0, -0.1480, 0.0790]
    q0 = [0.0, 0.0, 0.0, 0.0] # initial pose and guess
    amplitude = [0.05, 0.05, 0] # x, y and z direction
    f1 = 5; f2 = 5; f3 = 0 # Hz
    frequency = [2*1.57*f1, 2*1.57*f2, 2*1.57*f3]
    
    q1list = []
    q2list = []
    q3list = []
    q4list = []

    duration = 5.0
    max_time_step = 4000
    single_time_step = duration/max_time_step

    for i in range(max_time_step):
        x = offset[0] + amplitude[0]*cos(frequency[0]*i*single_time_step)
        y = offset[1] + amplitude[1]*sin(frequency[1]*i*single_time_step)
        z = offset[2] + amplitude[2]*cos(frequency[2]*i*single_time_step)

        q1, q2, q3 ,q4 = fsolve(pose_error, q0, args=[x,y,z])
        
        END = angle_to_pose(q1,q2,q3,q4)
        print("q1 : %f q2 : %f q3 : %f q4 : %f" % (q1, q2, q3, q4))
        print("X : %f Y : %f Z : %f" % (END[0],END[1],END[2]))

        q1list.append(q1)
        q2list.append(q2)
        q3list.append(q3)
        q4list.append(q4)
        
    
    rospy.wait_for_service('set_angle_list_service')

    try:
        set_angle_list = rospy.ServiceProxy('set_angle_list_service', SetAngleList)
        resp = set_angle_list(duration, max_time_step, q1list, q2list, q3list, q4list)
        if resp.success == True:
            rsp.success = True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        rsp.success = False
    return rsp

def manipulator_node():
    rospy.init_node('manipulator_node_node')
    rospy.Service('draw_circle', DrawCircle, draw_circle_callback)
    rospy.spin()

def openmanipulator_custom():
    for dxlHandler in dxlHandlerArray:
        # Open port
        dxlHandler.open_port()
        # Set port baudrate
        dxlHandler.set_baudrate()

    # start the manipulator node
    manipulator_node()

    for dxlHandler in dxlHandlerArray:
        # Close port
        dxlHandler.close_port()


def main():
    [x,y,z] = angle_to_pose(0.0, 0.0, 0.0, 0.0)
    print(x)
    print(y)
    print(z)
    openmanipulator_custom()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass