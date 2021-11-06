#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Jungill kang 
# 2021.11.07

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
from open_manipulator_custom.srv import SetAngle, SetAngleResponse


# dynamixel handler
dxlHandlerArray = []
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 11, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 12, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 13, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 14, DEVICENAME = '/dev/ttyUSB0'))
dxlHandlerArray.append(DxlHandler(PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 15, DEVICENAME = '/dev/ttyUSB0'))


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

def set_angle_node():
    rospy.init_node('set_angle_node')
    print("set_angle service on")
    rospy.Service('set_angle_service', SetAngle, set_angle_callback)
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