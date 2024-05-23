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
ADDR_ERROR_STATUS           = 518
LEN_ERROR_STATUS            = 1          # Data Byte Length
BAUDRATE                    = 2000000
ADDR_DRIVE_MODE             = 10

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
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
dxl_goal_position = [0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 5, 6, 8, 9, 10, 12, 13, 15, 16, 18, 20, 22, 24, 26, 28, 30, 33, 35, 38, 41, 43, 46, 49, 52, 55, 58, 62, 65, 69, 72, 76, 80, 83, 87, 91, 95, 100, 104, 108, 113, 117, 122, 127, 131, 136, 141, 146, 152, 157, 162, 168, 173, 179, 185, 191, 196, 202, 208, 215, 221, 227, 234, 240, 247, 254, 260, 267, 274, 281, 289, 296, 303, 311, 318, 326, 333, 341, 349, 357, 365, 373, 382, 390, 398, 407, 416, 424, 433, 442, 451, 460, 469, 478, 488, 497, 507, 516, 526, 536, 546, 556, 566, 576, 586, 596, 607, 617, 628, 639, 649, 660, 671, 682, 693, 705, 716, 727, 739, 750, 762, 774, 786, 798, 810, 822, 834, 846, 859, 871, 884, 896, 909, 922, 935, 948, 961, 974, 988, 1001, 1015, 1028, 1042, 1056, 1069, 1083, 1097, 1111, 1126, 1140, 1154, 1169, 1183, 1198, 1213, 1228, 1243, 1258, 1273, 1288, 1303, 1318, 1333, 1348, 1363, 1378, 1393, 1408, 1423, 1438, 1453, 1468, 1483, 1498, 1513, 1528, 1543, 1558, 1573, 1588, 1603, 1618, 1633, 1648, 1663, 1678, 1693, 1708, 1723, 1738, 1753, 1768, 1783, 1798, 1813, 1828, 1843, 1858, 1873, 1888, 1903, 1918, 1933, 1948, 1963, 1978, 1993, 2008, 2023, 2038, 2053, 2068, 2083, 2098, 2113, 2128, 2143, 2158, 2173, 2188, 2203, 2218, 2233, 2248, 2264, 2279, 2294, 2309, 2324, 2339, 2354, 2369, 2384, 2399, 2414, 2429, 2444, 2459, 2474, 2489, 2504, 2519, 2534, 2549, 2564, 2579, 2594, 2609, 2624, 2639, 2654, 2669, 2684, 2699, 2714, 2729, 2744, 2759, 2774, 2789, 2804, 2819, 2834, 2849, 2864, 2879, 2894, 2909, 2924, 2939, 2954, 2969, 2984, 2999, 3014, 3029, 3044, 3059, 3074, 3089, 3104, 3119, 3134, 3149, 3164, 3179, 3194, 3209, 3224, 3239, 3255, 3270, 3285, 3300, 3315, 3330, 3345, 3360, 3375, 3390, 3405, 3420, 3435, 3450, 3465, 3480, 3495, 3510, 3525, 3540, 3555, 3570, 3585, 3600, 3615, 3630, 3645, 3660, 3675, 3690, 3705, 3720, 3735, 3750, 3765, 3780, 3795, 3809, 3824, 3838, 3853, 3867, 3881, 3896, 3910, 3924, 3938, 3951, 3965, 3979, 3992, 4006, 4019, 4032, 4045, 4059, 4071, 4084, 4097, 4110, 4123, 4135, 4148, 4160, 4172, 4184, 4196, 4208, 4220, 4232, 4244, 4255, 4267, 4278, 4290, 4301, 4312, 4323, 4334, 4345, 4356, 4367, 4377, 4388, 4398, 4409, 4419, 4429, 4439, 4449, 4459, 4469, 4479, 4489, 4498, 4508, 4517, 4526, 4535, 4545, 4554, 4563, 4571, 4580, 4589, 4597, 4606, 4614, 4622, 4631, 4639, 4647, 4655, 4663, 4670, 4678, 4686, 4693, 4701, 4708, 4715, 4722, 4729, 4736, 4743, 4750, 4756, 4763, 4769, 4776, 4782, 4788, 4795, 4801, 4807, 4812, 4818, 4824, 4829, 4835, 4840, 4846, 4851, 4856, 4861, 4866, 4871, 4876, 4880, 4885, 4890, 4894, 4898, 4903, 4907, 4911, 4915, 4919, 4922, 4926, 4930, 4933, 4937, 4940, 4943, 4946, 4949, 4952, 4955, 4958, 4961, 4963, 4966, 4968, 4971, 4973, 4975, 4977, 4979, 4981, 4983, 4985, 4986, 4988, 4989, 4991, 4992, 4993, 4994, 4995, 4996, 4997, 4997, 4998, 4999, 4999, 4999, 5000, 5000, 5000, 5000, 5000, 5000, 4999, 4999, 4999, 4998, 4997, 4997, 4996, 4995, 4994, 4993, 4992, 4991, 4989, 4988, 4986, 4985, 4983, 4981, 4979, 4977, 4975, 4973, 4971, 4968, 4966, 4963, 4961, 4958, 4955, 4952, 4949, 4946, 4943, 4940, 4937, 4933, 4930, 4926, 4922, 4919, 4915, 4911, 4907, 4903, 4898, 4894, 4890, 4885, 4880, 4876, 4871, 4866, 4861, 4856, 4851, 4846, 4840, 4835, 4829, 4824, 4818, 4812, 4807, 4801, 4795, 4788, 4782, 4776, 4769, 4763, 4756, 4750, 4743, 4736, 4729, 4722, 4715, 4708, 4701, 4693, 4686, 4678, 4670, 4663, 4655, 4647, 4639, 4631, 4622, 4614, 4606, 4597, 4589, 4580, 4571, 4563, 4554, 4545, 4535, 4526, 4517, 4508, 4498, 4489, 4479, 4469, 4459, 4449, 4439, 4429, 4419, 4409, 4398, 4388, 4377, 4367, 4356, 4345, 4334, 4323, 4312, 4301, 4290, 4278, 4267, 4255, 4244, 4232, 4220, 4208, 4196, 4184, 4172, 4160, 4148, 4135, 4123, 4110, 4097, 4084, 4071, 4059, 4045, 4032, 4019, 4006, 3992, 3979, 3965, 3951, 3938, 3924, 3910, 3896, 3881, 3867, 3853, 3838, 3824, 3809, 3795, 3780, 3765, 3750, 3735, 3720, 3705, 3690, 3675, 3660, 3645, 3630, 3615, 3600, 3585, 3570, 3555, 3540, 3525, 3510, 3495, 3480, 3465, 3450, 3435, 3420, 3405, 3390, 3375, 3360, 3345, 3330, 3315, 3300, 3285, 3270, 3255, 3239, 3224, 3209, 3194, 3179, 3164, 3149, 3134, 3119, 3104, 3089, 3074, 3059, 3044, 3029, 3014, 2999, 2984, 2969, 2954, 2939, 2924, 2909, 2894, 2879, 2864, 2849, 2834, 2819, 2804, 2789, 2774, 2759, 2744, 2729, 2714, 2699, 2684, 2669, 2654, 2639, 2624, 2609, 2594, 2579, 2564, 2549, 2534, 2519, 2504, 2489, 2474, 2459, 2444, 2429, 2414, 2399, 2384, 2369, 2354, 2339, 2324, 2309, 2294, 2279, 2264, 2248, 2233, 2218, 2203, 2188, 2173, 2158, 2143, 2128, 2113, 2098, 2083, 2068, 2053, 2038, 2023, 2008, 1993, 1978, 1963, 1948, 1933, 1918, 1903, 1888, 1873, 1858, 1843, 1828, 1813, 1798, 1783, 1768, 1753, 1738, 1723, 1708, 1693, 1678, 1663, 1648, 1633, 1618, 1603, 1588, 1573, 1558, 1543, 1528, 1513, 1498, 1483, 1468, 1453, 1438, 1423, 1408, 1393, 1378, 1363, 1348, 1333, 1318, 1303, 1288, 1273, 1258, 1243, 1228, 1213, 1198, 1183, 1169, 1154, 1140, 1126, 1111, 1097, 1083, 1069, 1056, 1042, 1028, 1015, 1001, 988, 974, 961, 948, 935, 922, 909, 896, 884, 871, 859, 846, 834, 822, 810, 798, 786, 774, 762, 750, 739, 727, 716, 705, 693, 682, 671, 660, 649, 639, 628, 617, 607, 596, 586, 576, 566, 556, 546, 536, 526, 516, 507, 497, 488, 478, 469, 460, 451, 442, 433, 424, 416, 407, 398, 390, 382, 373, 365, 357, 349, 341, 333, 326, 318, 311, 303, 296, 289, 281, 274, 267, 260, 254, 247, 240, 234, 227, 221, 215, 208, 202, 196, 191, 185, 179, 173, 168, 162, 157, 152, 146, 141, 136, 131, 127, 122, 117, 113, 108, 104, 100, 95, 91, 87, 83, 80, 76, 72, 69, 65, 62, 58, 55, 52, 49, 46, 43, 41, 38, 35, 33, 30, 28, 26, 24, 22, 20, 18, 16, 15, 13, 12, 10, 9, 8, 6, 5, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0]
present_position = [0, 0, 0, 0, 0, 0];         # Present position
error_status = [0, 0, 0, 0, 0, 0];         # Hardware error status

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
groupSyncReadStatus = GroupSyncRead(portHandler, packetHandler, ADDR_ERROR_STATUS, LEN_ERROR_STATUS)

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
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[i], ADDR_GOAL_POSITION, dxl_goal_position[0])
    
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

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], dxl_goal_position[0], dxl_present_position))

        if not abs(dxl_goal_position[0] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
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
        
    # Add parameter storage for Dynamixel present position value
    dxl_addparam_result = groupSyncReadStatus.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncReadStatus addparam failed" % DXL_ID[i])
        quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    for index in range(0, len(dxl_goal_position)):
        # Syncread Error Status
        dxl_comm_result = groupSyncReadStatus.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            for i in range(0, len(DXL_ID)):
                error_status[i] = groupSyncReadStatus.getData(DXL_ID[i], ADDR_ERROR_STATUS, LEN_ERROR_STATUS)
                # print("error_status[%03d]:%03d " % (DXL_ID[i],error_status[i]))

        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            for i in range(0, len(DXL_ID)):
                present_position[i] = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                present_position[i] = struct.unpack('i', struct.pack('I', present_position[i]))[0]
                # print("present_position[%03d]:%03d " % (DXL_ID[i],present_position[i]))

        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]

        for i in range(0, len(DXL_ID)):
            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        # Wait for movement to goal position
        time.sleep(0.001)

# Clear syncread parameter storage
groupSyncRead.clearParam()

for i in range(0, len(DXL_ID)):
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
