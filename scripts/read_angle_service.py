#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Jungill kang 
# 2021.11.07

#######################################################################################
## ROS read angle of Dynamixel
## 
## USAGE : rosrun open_manipulator_custom read_angle_service.py
##         rosservice call /read_angle_service std_msgs/Empty "{}"
##         
##
## Service
##      /read_angle_service --> read all 5 angle data from openmanipulator and send back
##
## 
#######################################################################################

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
import os

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
from open_manipulator_custom.srv import ReadAngle, ReadAngleResponse


# dynamixel handler
dxlHandlerArray = []
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 11, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 12, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 13, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 14, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 15, DEVICENAME = '/dev/ttyUSB0'))


def read_angle_callback(req):
    rsp = ReadAngleResponse()
    
    angleList = []
    for idx, dxlHandler in enumerate(dxlHandlerArray):
        # Try to ping the Dynamixel
        dxlHandler.ping()

        # check with led
        dxlHandler.led_on()

        # read current angle
        currentAngle = dxlHandler.read_angle()

        # read angle from dynamxiel
        angleList.append(currentAngle)
    
    rsp.position1 = angleList[0]
    rsp.position2 = angleList[1]
    rsp.position3 = angleList[2]
    rsp.position4 = angleList[3]
    rsp.position5 = angleList[4]

    # wait and turn off the led
    
    for dxlHandler in dxlHandlerArray:
        # check with led
        dxlHandler.led_off()

    return rsp

def read_angle_node():
    rospy.init_node('read_angle_node')
    print("angle service on")
    rospy.Service('read_angle_service', ReadAngle, read_angle_callback)
    # rospy.Publisher('current_angle', Angle, read_angle_callback)
    rospy.spin()

def read_angle_service():
    for dxlHandler in dxlHandlerArray:
        # Open port
        dxlHandler.open_port()
        # Set port baudrate
        dxlHandler.set_baudrate()

    # start the ping node
    read_angle_node()

    for dxlHandler in dxlHandlerArray:
        # Close port
        dxlHandler.close_port()

def main():
    read_angle_service()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass