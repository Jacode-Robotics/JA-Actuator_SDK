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
ADDR_DRIVE_MODE             = 10;
ADDR_OPERATING_MODE         = 11;

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
Master_ID = [1,2]
Slave_ID = [3,4]


# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'


TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
PROFILE_ENABLE              = 0x0;              # Value for enable trajectory profile
PROFILE_DISABLE             = 0x02;             # Value for disable trajectory profile
CURRENT_CONTROL_MODE        = 0;                # Value for current control mode

dxl_goal_position = [0, 0, 0, 0, 0, 0];         # Goal position
dxl_present_position = [0, 0, 0, 0, 0, 0];         # Present position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()   

# Slave Robot Homing
for i in range(0, len(Slave_ID)):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, Slave_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % Slave_ID[i])

    # Enable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, Slave_ID[i], ADDR_DRIVE_MODE, PROFILE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write start position point
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, Slave_ID[i], ADDR_GOAL_POSITION, dxl_goal_position[i])
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position[i], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, Slave_ID[i], ADDR_PRESENT_POSITION)
        dxl_present_position[i] = struct.unpack('i', struct.pack('I', dxl_present_position[i]))[0];
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (Slave_ID[i], dxl_goal_position[i], dxl_present_position[i]))

        if not abs(dxl_goal_position[i] - dxl_present_position[i]) > DXL_MOVING_STATUS_THRESHOLD:
            break  

    # Disable slave motor trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, Slave_ID[i], ADDR_DRIVE_MODE, PROFILE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


# Master Robot Homing
for i in range(0, len(Master_ID)):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, Master_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % Master_ID[i])

    # Enable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, Master_ID[i], ADDR_DRIVE_MODE, PROFILE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write start position point
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, Master_ID[i], ADDR_GOAL_POSITION, dxl_goal_position[i])
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position[i], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, Master_ID[i], ADDR_PRESENT_POSITION)
        dxl_present_position[i] = struct.unpack('i', struct.pack('I', dxl_present_position[i]))[0];
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (Master_ID[i], dxl_goal_position[i], dxl_present_position[i]))

        if not abs(dxl_goal_position[i] - dxl_present_position[i]) > DXL_MOVING_STATUS_THRESHOLD:
            break    

    # Configure master motor operating mode to current control
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, Master_ID[i], ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))     

# Add parameter storage for Master Robot present position value
for i in range(0, len(Master_ID)):
    dxl_addparam_result = groupSyncRead.addParam(Master_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % Master_ID[i])
        quit()

while 1:
    # Syncread present position
    dxl_comm_result = groupSyncRead.txRxPacket()

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        for i in range(0, len(Master_ID)):
            dxl_present_position[i] = groupSyncRead.getData(Master_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl_present_position[i] = struct.unpack('i', struct.pack('I', dxl_present_position[i]))[0]
            if abs(dxl_present_position[i] - dxl_goal_position[i]) < 10000:
                dxl_goal_position[i] = dxl_present_position[i]
            print("dxl_goal_position[%03d]:%03d " % (Master_ID[i],dxl_goal_position[i]))

            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]
        
            # Add Salve Robot goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(Slave_ID[i], param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % Slave_ID[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

    # Wait for movement to goal position
    time.sleep(0.01)

# Close port
portHandler.closePort()
