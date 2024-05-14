#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import os
import struct
import time

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
ADDR_GOAL_POSITION          = 564
LEN_GOAL_POSITION           = 4          # Data Byte Length
ADDR_PRESENT_POSITION       = 580
LEN_PRESENT_POSITION        = 4          # Data Byte Length
BAUDRATE                    = 2000000
ADDR_DRIVE_MODE             = 10

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
# DXL_ID = [1, 2, 3, 4, 5, 6]
DXL_ID = [1, 2, 3, 4, 5, 6]

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
PROFILE_ENABLE              = 0x0;              # Value for enable trajectory profile
PROFILE_DISABLE             = 0x02;             # Value for disable trajectory profile

index = 0
dxl_goal_position = [0, 0, 0, 0, 0, 0]
dxl_present_position = [0, 0, 0, 0, 0, 0]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance for Position
groupSyncWritePosition = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncReadPosition = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

def Open_port(baudrate):
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(baudrate):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
Open_port(BAUDRATE)

for i in range(0, len(DXL_ID)):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[i], 560, 800)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID[%d] Write profile velocity success" % DXL_ID[i])

for i in range(0, len(DXL_ID)):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID[%d] Enable Dynamixel Torque success" % DXL_ID[i])

    # Enable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_DRIVE_MODE, PROFILE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))        
    
    # Add parameter storage for Dynamixel present position value
    dxl_addparam_result = groupSyncReadPosition.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncReadPosition addparam failed" % DXL_ID[i])
        quit()

def Send_Dynamixel_Position(Position):
    print("Testing position 1!")
    for i in range(0,len(DXL_ID)):
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(Position[i])), DXL_HIBYTE(DXL_LOWORD(Position[i])), DXL_LOBYTE(DXL_HIWORD(Position[i])), DXL_HIBYTE(DXL_HIWORD(Position[i]))]
        # Add Dynamixel goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWritePosition.addParam(DXL_ID[i], param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWritePosition addparam failed" % DXL_ID[i])
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWritePosition.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    else:
        print("Send_Dynamixel_Position success")

    # Clear syncwrite parameter storage
    groupSyncWritePosition.clearParam()


def Read_Dynamixel_Position():
    # syncread present position
    dxl_comm_result = groupSyncReadPosition.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    else:
        for i in range(0,len(DXL_ID)):
            dxl_present_position[i] = groupSyncReadPosition.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl_present_position[i] = struct.unpack('i', struct.pack('I', dxl_present_position[i]))[0]
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], dxl_goal_position[i], dxl_present_position[i]))

def Waiting_to_reach_goal():
    while 1:
        num = 0

        Read_Dynamixel_Position()

        for i in range(0,len(DXL_ID)):
            if abs(dxl_present_position[i] - dxl_goal_position[i]) <= DXL_MOVING_STATUS_THRESHOLD:
                num += 1
        if num == 6:
            break

while 1:
    pointion1 = [0, 0, 0, 0, 0, 0]
    for i in range(0,len(pointion1)):
        dxl_goal_position[i] = int(float(pointion1[i])  / 360 * pow(2 , 15))
    Send_Dynamixel_Position(dxl_goal_position)
    Waiting_to_reach_goal()
    time.sleep(1)

    pointion1 = [0, 0, 90, 0, 90, 0]
    for i in range(0,len(pointion1)):
        dxl_goal_position[i] = int(float(pointion1[i])  / 360 * pow(2 , 15))
    Send_Dynamixel_Position(dxl_goal_position)
    Waiting_to_reach_goal()
    time.sleep(1)

    pointion1 = [0, 20, 90, 0, 90, 0]
    for i in range(0,len(pointion1)):
        dxl_goal_position[i] = int(float(pointion1[i])  / 360 * pow(2 , 15))
    Send_Dynamixel_Position(dxl_goal_position)
    Waiting_to_reach_goal()
    time.sleep(6)

    pointion1 = [0, 0, 90, 0, 90, 0]
    for i in range(0,len(pointion1)):
        dxl_goal_position[i] = int(float(pointion1[i])  / 360 * pow(2 , 15))
    Send_Dynamixel_Position(dxl_goal_position)
    Waiting_to_reach_goal()
    time.sleep(1)

groupSyncReadPosition.clearParam()
# Close port
portHandler.closePort()