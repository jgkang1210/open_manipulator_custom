#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Jungill kang 
# 2021.11.07

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
import os
from math import cos, sin
from numpy import deg2rad, rad2deg

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
from open_manipulator_custom.srv import InitAngle, InitAngleResponse
from open_manipulator_custom.srv import SetAngle, SetAngleResponse
from open_manipulator_custom.srv import SetAngleList, SetAngleListResponse
from open_manipulator_custom.srv import CloseGripper, CloseGripperResponse
from open_manipulator_custom.srv import ReadAngle, ReadAngleResponse

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


def init_angle_callback(req):
    rsp = InitAngleResponse()
    
    angleList = [0,180,180,90,0]

    angleList[0] = req.position1
    angleList[1] = req.position2
    angleList[2] = req.position3
    angleList[3] = req.position4
    angleList[4] = req.position5

    for idx, dxlHandler in enumerate(dxlHandlerArray):
        # check with led
        dxlHandler.led_on()

        # read current angle
        success = dxlHandler.set_angle(angleList[idx])

        # read angle from dynamxiel
        rsp.success = success

    # wait and turn off the led
    for dxlHandler in dxlHandlerArray:
        # check with led
        dxlHandler.led_off()

    return rsp


def set_angle_callback(req):
    rsp = SetAngleResponse()
    
    angleList = [0,180,180,90,0]

    angleList[0] = req.position1
    angleList[1] = req.position2
    angleList[2] = req.position3
    angleList[3] = req.position4
    angleList[4] = req.position5

    for idx, dxlHandler in enumerate(dxlHandlerArray):
        # check with led
        dxlHandler.led_on()

        # read current angle
        success = dxlHandler.set_angle(angleList[idx])

        # read angle from dynamxiel
        rsp.success = success

    # wait and turn off the led
    for dxlHandler in dxlHandlerArray:
        # check with led
        dxlHandler.led_off()

    return rsp

def set_angle_list_callback(req):
    rsp = SetAngleListResponse()
    
    offsetList = [90,180,180,90,0]

    q1List = req.q1list
    q2List = req.q2list
    q3List = req.q3list
    q4List = req.q4list
    q5List = 0

    print("set_angle_list service starts!")

    duration = req.duration
    max_time_step = req.max_time_step
    single_time_step = duration/float(max_time_step)

    for dxlHandler in dxlHandlerArray:
            # check with led
            dxlHandler.led_on()

    for t in range(max_time_step):        
        angleList = [q1List[t], q2List[t], q3List[t], q4List[t], q5List]
        for idx, dxlHandler in enumerate(dxlHandlerArray):
            # read current angle
            success = dxlHandler.set_angle(rad2deg(angleList[idx])+offsetList[idx])
            # read angle from dynamxiel
            rsp.success = success
            # END = angle_to_pose(q1List[t], q2List[t], q3List[t], q4List[t])
            rospy.sleep(single_time_step/2)

    # wait and turn off the led
    for dxlHandler in dxlHandlerArray:
        # check with led
        dxlHandler.led_off()

    return rsp

def close_gripper_callback(req):
    gripper = 20

    print(req)
    
    if req.close == True:
        gripper = 20
    elif req.close == False:
        gripper = 80

    print(gripper)


    rsp = CloseGripperResponse()
    
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


    angleList = [0,180,180,90,0]

    angleList[0] = q0[0]
    angleList[1] = q0[1]
    angleList[2] = q0[2]
    angleList[3] = q0[3]
    angleList[4] = gripper

    for idx, dxlHandler in enumerate(dxlHandlerArray):
        # check with led
        dxlHandler.led_on()

        # read current angle
        success = dxlHandler.set_angle(angleList[idx])

        # read angle from dynamxiel
        rsp.success = success

    # wait and turn off the led
    for dxlHandler in dxlHandlerArray:
        # check with led
        dxlHandler.led_off()

    return rsp

def set_angle_node():
    rospy.init_node('set_angle_node')
    print("set_angle service on")
    rospy.Service('init_angle_service', InitAngle, init_angle_callback)
    rospy.Service('set_angle_service', SetAngle, set_angle_callback)
    rospy.Service('close_gripper_service', CloseGripper, close_gripper_callback)
    rospy.Service('set_angle_list_service', SetAngleList, set_angle_list_callback)
    # rospy.Publisher('current_angle', Angle, read_angle_callback)
    rospy.spin()

def set_angle_service():
    for dxlHandler in dxlHandlerArray:
        # Open port
        dxlHandler.open_port()
        # Set port baudrate
        dxlHandler.set_baudrate()
        # Set torque enable
        dxlHandler.torque_enable()

    # start the ping node
    set_angle_node()

    for dxlHandler in dxlHandlerArray:
        # Close port
        dxlHandler.close_port()
        # Set torque disable
        dxlHandler.torque_disable()

def main():
    set_angle_service()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass