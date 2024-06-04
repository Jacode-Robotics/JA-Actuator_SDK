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
import ctypes

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
DXL_MINIMUM_POSITION_VALUE  = -5000      # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 5000       # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 2000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL_ID = [1,3]


# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM15'


TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
SOURCE                      = 0
index = 0
#dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, 36, 2)

def Power_judgment():

    for i in range(1,30):
        dxl_model_number,dxl_comm_result, dxl_error = packetHandler.ping(portHandler, i)
        if dxl_comm_result != COMM_SUCCESS:
            continue
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            global SOURCE
            SOURCE = 1
            print("Succeeded to open the port")
            print("Succeeded to change the baudrate") 
            break

def is_port_online(port_name):
    if os.name == 'nt':
        device_path = f"\\\\.\\{port_name}"
        handle = ctypes.windll.kernel32.CreateFileW(device_path,0x80000000,0,None,3,0x10000000,None)
        if handle != -1:
            ctypes.windll.kernel32.CloseHandle(handle)
            return True
        else:
            return False
    else:
        device_path = f"{port_name}"
        return os.path.exists(device_path)

if is_port_online(DEVICENAME):
    print(f"{DEVICENAME} on line")
else:
    print(f"{DEVICENAME} not online")

# Open port
if portHandler.openPort() == 0:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE) == 0:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

Power_judgment()

if SOURCE:

    for i in range(0, len(DXL_ID)):
        # Add parameter storage for Dynamixel present position value
        dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID[i])
            quit()

        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        for i in range(0, len(DXL_ID)):
            # Get Dynamixel present position value
            pwmlimit = groupSyncRead.getData(DXL_ID[i], 36, 2)
            pwmlimit = struct.unpack('i', struct.pack('I', pwmlimit))[0];

            print("[ID:%03d] PresPos:%03d" % (DXL_ID[i] ,pwmlimit))

    # Close port
    portHandler.closePort()

else:
    print("Not powered on/not connected to equipment")