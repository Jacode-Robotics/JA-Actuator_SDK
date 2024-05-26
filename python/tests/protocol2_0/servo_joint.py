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
ADDR_GOAL_VELOCITY          = 552
LEN_GOAL_VELOCITY           = 4          # Data Byte Length
BAUDRATE                    = 2000000
ADDR_DRIVE_MODE             = 10

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
# DXL_ID = [1, 2, 3, 4, 5, 6]
DXL_ID = [1]



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
# dxl_goal_position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -2, -3, -5, -6, -9, -11, -15, -18, -22, -27, -32, -38, -45, -52, -60, -69, -78, -88, -99, -111, -123, -136, -151, -166, -182, -198, -216, -235, -254, -275, -296, -319, -342, -366, -392, -418, -445, -474, -503, -533, -564, -630, -664, -699, -735, -772, -810, -848, -888, -929, -970, -1013, -1056, -1100, -1145, -1191, -1238, -1285, -1333, -1382, -1432, -1482, -1533, -1585, -1637, -1690, -1743, -1797, -1852, -1906, -1962, -2017, -2130, -2187, -2244, -2301, -2358, -2416, -2473, -2531, -2588, -2646, -2704, -2762, -2820, -2877, -2935, -2993, -3051, -3109, -3167, -3224, -3282, -3340, -3398, -3455, -3513, -3570, -3628, -3685, -3742, -3800, -3857, -3914, -3970, -4027, -4084, -4140, -4196, -4253, -4309, -4364, -4420, -4475, -4586, -4641, -4695, -4750, -4804, -4858, -4912, -4965, -5018, -5071, -5124, -5176, -5229, -5280, -5332, -5383, -5434, -5485, -5535, -5585, -5635, -5684, -5733, -5781, -5830, -5877, -5925, -5972, -6018, -6065, -6111, -6156, -6201, -6246, -6290, -6333, -6376, -6419, -6462, -6503, -6545, -6626, -6666, -6666, -6744, -6782, -6820, -6857, -6894, -6930, -6965, -7000, -7035, -7068, -7102, -7134, -7166, -7198, -7228, -7258, -7288, -7317, -7345, -7373, -7400, -7426, -7451, -7476, -7501, -7525, -7548, -7570, -7592, -7614, -7635, -7655, -7694, -7713, -7731, -7748, -7766, -7782, -7798, -7814, -7830, -7844, -7859, -7873, -7886, -7899, -7912, -7924, -7936, -7948, -7959, -7969, -7980, -7990, -7999, -8009, -8018, -8026, -8035, -8043, -8050, -8058, -8065, -8072, -8078, -8085, -8091, -8096, -8102, -8107, -8112, -8117, -8122, -8126, -8130, -8134, -8138, -8145, -8148, -8151, -8154, -8157, -8159, -8162, -8164, -8166, -8168, -8170, -8172, -8173, -8175, -8176, -8177, -8178, -8180, -8181, -8181, -8182, -8183, -8184, -8184, -8185, -8185, -8186, -8186, -8186, -8186, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8187, -8186, -8186, -8186, -8186, -8185, -8185, -8184, -8184, -8183, -8182, -8181, -8181, -8180, -8178, -8177, -8176, -8175, -8173, -8172, -8170, -8168, -8166, -8164, -8162, -8159, -8157, -8154, -8151, -8148, -8145, -8138, -8134, -8130, -8126, -8122, -8117, -8112, -8107, -8102, -8096, -8091, -8085, -8078, -8072, -8065, -8058, -8050, -8043, -8035, -8026, -8018, -8009, -7999, -7990, -7980, -7969, -7959, -7948, -7936, -7924, -7912, -7899, -7886, -7873, -7859, -7844, -7830, -7814, -7798, -7782, -7766, -7748, -7731, -7713, -7694, -7655, -7635, -7614, -7592, -7570, -7548, -7525, -7501, -7476, -7451, -7426, -7400, -7373, -7345, -7317, -7288, -7258, -7228, -7198, -7166, -7134, -7102, -7068, -7035, -7000, -6965, -6930, -6894, -6857, -6820, -6782, -6744, -6666, -6666, -6626, -6545, -6503, -6462, -6419, -6376, -6333, -6290, -6246, -6201, -6156, -6111, -6065, -6018, -5972, -5925, -5877, -5830, -5781, -5733, -5684, -5635, -5585, -5535, -5485, -5434, -5383, -5332, -5280, -5229, -5176, -5124, -5071, -5018, -4965, -4912, -4858, -4804, -4750, -4695, -4641, -4586, -4475, -4420, -4364, -4309, -4253, -4196, -4140, -4084, -4027, -3970, -3914, -3857, -3800, -3742, -3685, -3628, -3570, -3513, -3455, -3398, -3340, -3282, -3224, -3167, -3109, -3051, -2993, -2935, -2877, -2820, -2762, -2704, -2646, -2588, -2531, -2473, -2416, -2358, -2301, -2244, -2187, -2130, -2017, -1962, -1906, -1852, -1797, -1743, -1690, -1637, -1585, -1533, -1482, -1432, -1382, -1333, -1285, -1238, -1191, -1145, -1100, -1056, -1013, -970, -929, -888, -848, -810, -772, -735, -699, -664, -630, -564, -533, -503, -474, -445, -418, -392, -366, -342, -319, -296, -275, -254, -235, -216, -198, -182, -166, -151, -136, -123, -111, -99, -88, -78, -69, -60, -52, -45, -38, -32, -27, -22, -18, -15, -11, -9, -6, -5, -3, -2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
dxl_goal_position = [0, 0, -1, -3, -5, -8, -12, -16, -21, -27, -33, -40, -48, -56, -65, -75, -85, -96, -107, -120, -133, -146, -160, -175, -191, -207, -224, -242, -260, -279, -298, -319, -340, -361, -383, -406, -430, -454, -479, -504, -530, -557, -585, -613, -642, -671, -702, -732, -764, -796, -829, -862, -897, -931, -967, -1003, -1040, -1077, -1115, -1154, -1194, -1234, -1275, -1316, -1358, -1401, -1444, -1488, -1533, -1579, -1625, -1671, -1719, -1767, -1816, -1865, -1915, -1966, -2017, -2069, -2120, -2172, -2223, -2275, -2326, -2378, -2429, -2481, -2532, -2584, -2635, -2687, -2738, -2790, -2841, -2893, -2944, -2996, -3047, -3099, -3150, -3202, -3253, -3305, -3356, -3408, -3459, -3511, -3562, -3614, -3665, -3717, -3768, -3820, -3871, -3923, -3974, -4026, -4077, -4129, -4180, -4232, -4283, -4335, -4386, -4438, -4489, -4541, -4592, -4644, -4695, -4747, -4798, -4850, -4901, -4953, -5004, -5056, -5107, -5159, -5210, -5262, -5313, -5365, -5416, -5468, -5519, -5571, -5622, -5674, -5725, -5777, -5828, -5880, -5931, -5983, -6034, -6085, -6135, -6184, -6233, -6281, -6329, -6375, -6421, -6467, -6512, -6556, -6599, -6642, -6684, -6725, -6766, -6806, -6846, -6885, -6923, -6960, -6997, -7033, -7069, -7103, -7138, -7171, -7204, -7236, -7268, -7298, -7329, -7358, -7387, -7415, -7443, -7470, -7496, -7521, -7546, -7570, -7594, -7617, -7639, -7660, -7681, -7702, -7721, -7740, -7758, -7776, -7793, -7809, -7825, -7840, -7854, -7867, -7880, -7893, -7904, -7915, -7925, -7935, -7944, -7952, -7960, -7967, -7973, -7979, -7984, -7988, -7992, -7995, -7997, -7999, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -8000, -7999, -7997, -7995, -7992, -7988, -7984, -7979, -7973, -7967, -7960, -7952, -7944, -7935, -7925, -7915, -7904, -7893, -7880, -7867, -7854, -7840, -7825, -7809, -7793, -7776, -7758, -7740, -7721, -7702, -7681, -7660, -7639, -7617, -7594, -7570, -7546, -7521, -7496, -7470, -7443, -7415, -7387, -7358, -7329, -7298, -7268, -7236, -7204, -7171, -7138, -7103, -7069, -7033, -6997, -6960, -6923, -6885, -6846, -6806, -6766, -6725, -6684, -6642, -6599, -6556, -6512, -6467, -6421, -6375, -6329, -6281, -6233, -6184, -6135, -6085, -6034, -5983, -5931, -5880, -5828, -5777, -5725, -5674, -5622, -5571, -5519, -5468, -5416, -5365, -5313, -5262, -5210, -5159, -5107, -5056, -5004, -4953, -4901, -4850, -4798, -4747, -4695, -4644, -4592, -4541, -4489, -4438, -4386, -4335, -4283, -4232, -4180, -4129, -4077, -4026, -3974, -3923, -3871, -3820, -3768, -3717, -3665, -3614, -3562, -3511, -3459, -3408, -3356, -3305, -3253, -3202, -3150, -3099, -3047, -2996, -2944, -2893, -2841, -2790, -2738, -2687, -2635, -2584, -2532, -2481, -2429, -2378, -2326, -2275, -2223, -2172, -2120, -2069, -2017, -1966, -1915, -1865, -1816, -1767, -1719, -1671, -1625, -1579, -1533, -1488, -1444, -1401, -1358, -1316, -1275, -1234, -1194, -1154, -1115, -1077, -1040, -1003, -967, -931, -897, -862, -829, -796, -764, -732, -702, -671, -642, -613, -585, -557, -530, -504, -479, -454, -430, -406, -383, -361, -340, -319, -298, -279, -260, -242, -224, -207, -191, -175, -160, -146, -133, -120, -107, -96, -85, -75, -65, -56, -48, -40, -33, -27, -21, -16, -12, -8, -5, -3, -1, 0, 0]

# dxl_goal_velocity = [0, -25, -75, -100, -125, -175, -200, -225, -275, -300, -325, -375, -400, -425, -475, -500, -525, -550, -600, -650, -650, -675, -725, -775, -800, -825, -875, -900, -925, -950, -1000, -1050, -1050, -1075, -1125, -1175, -1200, -1225, -1250, -1275, -1325, -1375, -1400, -1425, -1450, -1500, -1525, -1550, -1600, -1625, -1650, -1700, -1725, -1750, -1800, -1825, -1850, -1875, -1925, -1975, -2000, -2025, -2050, -2075, -2125, -2150, -2175, -2225, -2275, -2300, -2300, -2350, -2400, -2425, -2450, -2475, -2525, -2550, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2575, -2550, -2525, -2475, -2450, -2425, -2400, -2350, -2300, -2300, -2275, -2225, -2175, -2150, -2125, -2075, -2050, -2025, -2000, -1975, -1925, -1875, -1850, -1825, -1800, -1750, -1725, -1700, -1650, -1625, -1600, -1550, -1525, -1500, -1450, -1425, -1400, -1375, -1325, -1275, -1250, -1225, -1200, -1175, -1125, -1075, -1050, -1050, -1000, -950, -925, -900, -875, -825, -800, -775, -725, -675, -650, -650, -600, -550, -525, -500, -475, -425, -400, -375, -325, -300, -275, -225, -200, -175, -125, -100, -75, -25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 75, 100, 125, 175, 200, 225, 275, 300, 325, 375, 400, 425, 475, 500, 525, 550, 600, 650, 650, 675, 725, 775, 800, 825, 875, 900, 925, 950, 1000, 1050, 1050, 1075, 1125, 1175, 1200, 1225, 1250, 1275, 1325, 1375, 1400, 1425, 1450, 1500, 1525, 1550, 1600, 1625, 1650, 1700, 1725, 1750, 1800, 1825, 1850, 1875, 1925, 1975, 2000, 2025, 2050, 2075, 2125, 2150, 2175, 2225, 2275, 2300, 2300, 2350, 2400, 2425, 2450, 2475, 2525, 2550, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2575, 2550, 2525, 2475, 2450, 2425, 2400, 2350, 2300, 2300, 2275, 2225, 2175, 2150, 2125, 2075, 2050, 2025, 2000, 1975, 1925, 1875, 1850, 1825, 1800, 1750, 1725, 1700, 1650, 1625, 1600, 1550, 1525, 1500, 1450, 1425, 1400, 1375, 1325, 1275, 1250, 1225, 1200, 1175, 1125, 1075, 1050, 1050, 1000, 950, 925, 900, 875, 825, 800, 775, 725, 675, 650, 650, 600, 550, 525, 500, 475, 425, 400, 375, 325, 300, 275, 225, 200, 175, 125, 100, 75, 25, 0]
dxl_goal_velocity = [0, -6, -11, -17, -23, -28, -34, -40, -45, -51, -57, -62, -68, -74, -79, -85, -91, -96, -102, -108, -113, -119, -124, -130, -136, -141, -147, -153, -158, -164, -170, -175, -181, -187, -192, -198, -204, -209, -215, -221, -226, -232, -238, -243, -249, -255, -260, -266, -272, -277, -283, -289, -294, -300, -306, -311, -317, -323, -328, -334, -339, -345, -351, -356, -362, -368, -373, -379, -385, -390, -396, -402, -407, -413, -419, -424, -430, -436, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -439, -436, -430, -424, -419, -413, -407, -402, -396, -390, -385, -379, -373, -368, -362, -356, -351, -345, -339, -334, -328, -323, -317, -311, -306, -300, -294, -289, -283, -277, -272, -266, -260, -255, -249, -243, -238, -232, -226, -221, -215, -209, -204, -198, -192, -187, -181, -175, -170, -164, -158, -153, -147, -141, -136, -130, -124, -119, -113, -108, -102, -96, -91, -85, -79, -74, -68, -62, -57, -51, -45, -40, -34, -28, -23, -17, -11, -6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 11, 17, 23, 28, 34, 40, 45, 51, 57, 62, 68, 74, 79, 85, 91, 96, 102, 108, 113, 119, 124, 130, 136, 141, 147, 153, 158, 164, 170, 175, 181, 187, 192, 198, 204, 209, 215, 221, 226, 232, 238, 243, 249, 255, 260, 266, 272, 277, 283, 289, 294, 300, 306, 311, 317, 323, 328, 334, 339, 345, 351, 356, 362, 368, 373, 379, 385, 390, 396, 402, 407, 413, 419, 424, 430, 436, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 439, 436, 430, 424, 419, 413, 407, 402, 396, 390, 385, 379, 373, 368, 362, 356, 351, 345, 339, 334, 328, 323, 317, 311, 306, 300, 294, 289, 283, 277, 272, 266, 260, 255, 249, 243, 238, 232, 226, 221, 215, 209, 204, 198, 192, 187, 181, 175, 170, 164, 158, 153, 147, 141, 136, 130, 124, 119, 113, 108, 102, 96, 91, 85, 79, 74, 68, 62, 57, 51, 45, 40, 34, 28, 23, 17, 11, 6, 0]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWritePos = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
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

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    for index in range(0, len(dxl_goal_velocity)):
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]

        for i in range(0, len(DXL_ID)):
            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWritePos.addParam(DXL_ID[i], param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWritePos addparam failed" % DXL_ID[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWritePos.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWritePos.clearParam()
        time.sleep(0.002)

        # Allocate goal position value into byte array
        param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity[index]))]

        for i in range(0, len(DXL_ID)):
            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = groupSyncWriteVel.addParam(DXL_ID[i], param_goal_velocity)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWriteVel addparam failed" % DXL_ID[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWriteVel.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWriteVel.clearParam()

        # Wait for movement to goal position
        time.sleep(0.01)

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
