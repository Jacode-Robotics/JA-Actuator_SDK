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
import re

# Control table address
ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
ADDR_GOAL_POSITION          = 564
LEN_GOAL_POSITION           = 4          # Data Byte Length
ADDR_PRESENT_POSITION       = 580
LEN_PRESENT_POSITION        = 4          # Data Byte Length
ADDR_GOAL_VELOCITY          = 552
LEN_GOAL_VELOCITY           = 4          # Data Byte Length
BAUDRATE                    = 2000000
ADDR_DRIVE_MODE             = 10

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL_ID = [1, 2, 3, 4, 5, 6]
# DXL_ID = [1]



# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'


TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
PROFILE_ENABLE              = 0x0;              # Value for enable trajectory profile
PROFILE_DISABLE             = 0x02;             # Value for disable trajectory profile

index = 0
# dxl_goal_position = [0, 1, 5, 10, 18, 28, 41, 56, 73, 92, 114, 137, 164, 192, 223, 256, 291, 328, 368, 410, 455, 501, 550, 601, 655, 710, 768, 828, 891, 956, 1023, 1092, 1164, 1237, 1313, 1388, 1464, 1539, 1614, 1690, 1765, 1840, 1916, 1991, 2067, 2142, 2217, 2293, 2368, 2443, 2519, 2594, 2670, 2745, 2820, 2896, 2971, 3046, 3122, 3197, 3273, 3348, 3423, 3499, 3574, 3649, 3725, 3800, 3872, 3943, 4011, 4077, 4141, 4202, 4261, 4318, 4372, 4425, 4475, 4522, 4568, 4611, 4652, 4691, 4727, 4761, 4793, 4822, 4850, 4875, 4897, 4918, 4936, 4952, 4966, 4977, 4986, 4993, 4997, 5000, 5000, 4997, 4993, 4986, 4977, 4966, 4952, 4936, 4918, 4897, 4875, 4850, 4822, 4793, 4761, 4727, 4691, 4652, 4611, 4568, 4522, 4475, 4425, 4372, 4318, 4261, 4202, 4141, 4077, 4011, 3943, 3872, 3800, 3725, 3649, 3574, 3499, 3423, 3348, 3273, 3197, 3122, 3046, 2971, 2896, 2820, 2745, 2670, 2594, 2519, 2443, 2368, 2293, 2217, 2142, 2067, 1991, 1916, 1840, 1765, 1690, 1614, 1539, 1464, 1388, 1313, 1237, 1164, 1092, 1023, 956, 891, 828, 768, 710, 655, 601, 550, 501, 455, 410, 368, 328, 291, 256, 223, 192, 164, 137, 114, 92, 73, 56, 41, 28, 18, 10, 5, 1, 0]         # Goal position

def extract_data_from_jointld(file_path):
    extracted_data = []
    
    with open(file_path, 'r') as file:
        for line in file:
            # Match the pattern |index; followed by comma-separated float values
            match = re.match(r'\|(\d+);([\d\.\-\+,]+)', line.strip())
            if match:
                index = int(match.group(1))
                data_values = match.group(2).split(',')
                
                # Filter out any empty strings and convert the remaining data values to floats
                data_values = [float(val) for val in data_values if val]
                
                # Ensure we have exactly 12 values
                if len(data_values) == 12:
                    # Scale the positions and velocities and convert to integers
                    dxl_goal_position = [int((val / 360.0) * 32768) for val in data_values[:6]]
                    dxl_goal_velocity = [int((val / 360.0) * 32768) for val in data_values[6:12]]
                    
                    extracted_data.append({
                        'index': index,
                        'dxl_goal_position': dxl_goal_position,
                        'dxl_goal_velocity': dxl_goal_velocity
                    })
    
    return extracted_data

# Example usage
file_path = 'JointLD_25.data'
data = extract_data_from_jointld(file_path)
dxl_goal_position = data[0]['dxl_goal_position']

# Print the extracted data
# for entry in data:
#     print(f"Index: {entry['index']}")
#     print(f"Goal Positions: {entry['dxl_goal_position']}")
#     print(f"Goal Velocities: {entry['dxl_goal_velocity']}")
#     print('-' * 50)

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
groupSyncWriteVel = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

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

for i in range(0, len(DXL_ID)):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL_ID[i])

    # Enable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_DRIVE_MODE, PROFILE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write start position point
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[i], ADDR_GOAL_POSITION, dxl_goal_position[i])
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID[i], ADDR_PRESENT_POSITION)
        dxl_present_position = struct.unpack('i', struct.pack('I', dxl_present_position))[0];
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], dxl_goal_position[i], dxl_present_position))

        if not abs(dxl_goal_position[i] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break    

    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))       
        
    # Disable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_DRIVE_MODE, PROFILE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))        
    
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))       
            
    # Add parameter storage for Dynamixel present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID[i])
        quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    # for index in range(0, len(dxl_goal_position)):
    for entry in data:
        dxl_goal_position = entry['dxl_goal_position']
        dxl_goal_velocity = entry['dxl_goal_velocity']
        
        for i in range(0, len(DXL_ID)):
            # Allocate goal position value into byte array
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]

            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
                quit()

            # Allocate goal velocity value into byte array
            param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity[i])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity[i]))]

            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWriteVel.addParam(DXL_ID[i], param_goal_velocity)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWriteVel addparam failed" % DXL_ID[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Syncwrite goal velocity
        dxl_comm_result = groupSyncWriteVel.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
        groupSyncWriteVel.clearParam()

        # Wait for movement to goal position
        time.sleep(0.005)

# Clear syncread parameter storage
groupSyncRead.clearParam()

for i in range(len(DXL_ID), 0):
    # Enable trajectory profile
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_DRIVE_MODE, PROFILE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write home position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[i], ADDR_GOAL_POSITION, 0)
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID[i], ADDR_PRESENT_POSITION)
        dxl_present_position = struct.unpack('i', struct.pack('I', dxl_present_position))[0];
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], 0, dxl_present_position))

        if not abs(0 - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break    
        
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
