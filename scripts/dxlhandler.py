#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Jungill kang 
# 2021.11.07

from dynamixel_sdk import *                 # Uses Dynamixel SDK library

ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_LED = 65
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_POSITION = 116

class DxlHandler:
    def __init__(self, PROTOCOL_VERSION = 2.0, BAUDRATE = 1000000, DXL_ID = 11, DEVICENAME = '/dev/ttyUSB0'):
        self.PROTOCOL_VERSION = PROTOCOL_VERSION 
        self.BAUDRATE = BAUDRATE
        self.DXL_ID = DXL_ID # 11~15
        self.DEVICENAME = DEVICENAME
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
    
    def get_name(self):
        return self.DEVICENAME
    
    def set_baudrate(self):
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def open_port(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def torque_enable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print(dxl_comm_result)
            print("Failed to enable torque for Dynamixel ID: %d" % self.DXL_ID)

    def torque_disable(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("Failed to enable torque for Dynamixel ID: %d" % self.DXL_ID)

    # Import from robotis dynamixel sdk for reference
    
    #  def writeTxRx(self, port, dxl_id, address, length, data):
    #     txpacket = [0] * (length + 12)
    #     txpacket[PKT_ID] = dxl_id
    #     txpacket[PKT_LENGTH_L] = DXL_LOBYTE(length + 5)
    #     txpacket[PKT_LENGTH_H] = DXL_HIBYTE(length + 5)
    #     txpacket[PKT_INSTRUCTION] = INST_WRITE
    #     txpacket[PKT_PARAMETER0 + 0] = DXL_LOBYTE(address)
    #     txpacket[PKT_PARAMETER0 + 1] = DXL_HIBYTE(address)
    #
    #     txpacket[PKT_PARAMETER0 + 2: PKT_PARAMETER0 + 2 + length] = data[0: length]
    #     rxpacket, result, error = self.txRxPacket(port, txpacket)
    #
    #     return result, error
    
    # def write1ByteTxRx(self, port, dxl_id, address, data):
    #     data_write = [data]
    #     return self.writeTxRx(port, dxl_id, address, 1, data_write)

    # def write1ByteTxRx(self, port, dxl_id, address, data):
    #     data_write = [data]
    #     return self.writeTxRx(port, dxl_id, address, 1, data_write)

    def led_on(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_PRESENT_LED, 1)
        print("LED of ID %s = %s" % (self.DXL_ID, "1"))

    def led_off(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_PRESENT_LED, 0)
        print("LED of ID %s = %s" % (self.DXL_ID, "0"))

    def ping(self):
        # Get Dynamixel model number
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, self.DXL_ID)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.DXL_ID, dxl_model_number))

    # for XM430-W350-T
    def read_angle(self):
        dxl_cur_pwm, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, ADDR_PRESENT_POSITION)
        dxl_present_angle = self.pwm_to_degree(dxl_cur_pwm)
        print("Present Angle of ID %s = %s" % (self.DXL_ID, dxl_present_angle))
        return dxl_present_angle

    def set_angle(self, angle):
        dxl_goal_pwm = self.degree_to_pwm(angle)
        print(dxl_goal_pwm)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, ADDR_GOAL_POSITION, dxl_goal_pwm)
        print("Setpoint Angle of ID %s = %s" % (self.DXL_ID, angle))
        if dxl_comm_result == COMM_SUCCESS:
            return True
        else:
            return False

    def pwm_to_degree(self,pwm):
        angle = float(pwm/4095.0*360.0)
        return angle

    def degree_to_pwm(self,angle):
        pwm = int(float(int(angle)%360)/360*4095)
        return pwm

    def close_port(self):
        self.portHandler.closePort()

