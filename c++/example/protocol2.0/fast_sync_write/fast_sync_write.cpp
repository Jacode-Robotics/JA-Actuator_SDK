/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon), Honghyun Kim */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available DYNAMIXEL model on this example : All models using Protocol 2.0
// This example is tested with two DYNAMIXEL P Series, and an U2D2
// Be sure that DYNAMIXEL P properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <thread>

#include "dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          512                 // Control table address is different in DYNAMIXEL model
#define ADDR_PRO_GOAL_POSITION          564
#define ADDR_PRO_GOAL_VELOCITY          552

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_GOAL_VELOCITY           4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL

// Default setting
#define DXL1_ID                         1                   // DYNAMIXEL#1 ID: 1
#define DXL2_ID                         2                   // DYNAMIXEL#2 ID: 2
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupFastSyncWrite instance for Goal Position and Present Position
  dynamixel::GroupFastSyncWrite groupFastSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
  
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_VELOCITY, LEN_PRO_GOAL_VELOCITY);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result
  // int dxl_goal_position[] = {0, 1, 5, 10, 18, 28, 41, 56, 73, 92, 114, 137, 164, 192, 223, 256, 291, 328, 368, 410, 455, 501, 550, 601, 655, 710, 768, 828, 891, 956, 1023, 1092, 1164, 1237, 1313, 1388, 1464, 1539, 1614, 1690, 1765, 1840, 1916, 1991, 2067, 2142, 2217, 2293, 2368, 2443, 2519, 2594, 2670, 2745, 2820, 2896, 2971, 3046, 3122, 3197, 3273, 3348, 3423, 3499, 3574, 3649, 3725, 3800, 3872, 3943, 4011, 4077, 4141, 4202, 4261, 4318, 4372, 4425, 4475, 4522, 4568, 4611, 4652, 4691, 4727, 4761, 4793, 4822, 4850, 4875, 4897, 4918, 4936, 4952, 4966, 4977, 4986, 4993, 4997, 5000, 5000, 4997, 4993, 4986, 4977, 4966, 4952, 4936, 4918, 4897, 4875, 4850, 4822, 4793, 4761, 4727, 4691, 4652, 4611, 4568, 4522, 4475, 4425, 4372, 4318, 4261, 4202, 4141, 4077, 4011, 3943, 3872, 3800, 3725, 3649, 3574, 3499, 3423, 3348, 3273, 3197, 3122, 3046, 2971, 2896, 2820, 2745, 2670, 2594, 2519, 2443, 2368, 2293, 2217, 2142, 2067, 1991, 1916, 1840, 1765, 1690, 1614, 1539, 1464, 1388, 1313, 1237, 1164, 1092, 1023, 956, 891, 828, 768, 710, 655, 601, 550, 501, 455, 410, 368, 328, 291, 256, 223, 192, 164, 137, 114, 92, 73, 56, 41, 28, 18, 10, 5, 1, 0};
  int dxl_goal_position[] = {0, 0, 0, 0, -1, -1, -2, -2, -3, -4, -5, -5, -6, -8, -9, -10, -12, -13, -15, -16, -18, -20, -22, -24, -26, -28, -30, -33, -35, -38, -41, -43, -46, -49, -52, -55, -58, -62, -65, -69, -72, -76, -80, -83, -87, -91, -95, -100, -104, -108, -113, -117, -122, -127, -131, -136, -141, -146, -152, -157, -162, -168, -173, -179, -185, -191, -196, -202, -208, -215, -221, -227, -234, -240, -247, -254, -260, -267, -274, -281, -289, -296, -303, -311, -318, -326, -333, -341, -349, -357, -365, -373, -382, -390, -398, -407, -416, -424, -433, -442, -451, -460, -469, -478, -488, -497, -507, -516, -526, -536, -546, -556, -566, -576, -586, -596, -607, -617, -628, -639, -649, -660, -671, -682, -693, -705, -716, -727, -739, -750, -762, -774, -786, -798, -810, -822, -834, -846, -859, -871, -884, -896, -909, -922, -935, -948, -961, -974, -988, -1001, -1015, -1028, -1042, -1056, -1069, -1083, -1097, -1111, -1126, -1140, -1154, -1169, -1183, -1198, -1213, -1228, -1243, -1258, -1273, -1288, -1303, -1318, -1333, -1348, -1363, -1378, -1393, -1408, -1423, -1438, -1453, -1468, -1483, -1498, -1513, -1528, -1543, -1558, -1573, -1588, -1603, -1618, -1633, -1648, -1663, -1678, -1693, -1708, -1723, -1738, -1753, -1768, -1783, -1798, -1813, -1828, -1843, -1858, -1873, -1888, -1903, -1918, -1933, -1948, -1963, -1978, -1993, -2008, -2023, -2038, -2053, -2068, -2083, -2098, -2113, -2128, -2143, -2158, -2173, -2188, -2203, -2218, -2233, -2248, -2264, -2279, -2294, -2309, -2324, -2339, -2354, -2369, -2384, -2399, -2414, -2429, -2444, -2459, -2474, -2489, -2504, -2519, -2534, -2549, -2564, -2579, -2594, -2609, -2624, -2639, -2654, -2669, -2684, -2699, -2714, -2729, -2744, -2759, -2774, -2789, -2804, -2819, -2834, -2849, -2864, -2879, -2894, -2909, -2924, -2939, -2954, -2969, -2984, -2999, -3014, -3029, -3044, -3059, -3074, -3089, -3104, -3119, -3134, -3149, -3164, -3179, -3194, -3209, -3224, -3239, -3255, -3270, -3285, -3300, -3315, -3330, -3345, -3360, -3375, -3390, -3405, -3420, -3435, -3450, -3465, -3480, -3495, -3510, -3525, -3540, -3555, -3570, -3585, -3600, -3615, -3630, -3645, -3660, -3675, -3690, -3705, -3720, -3735, -3750, -3765, -3780, -3795, -3809, -3824, -3838, -3853, -3867, -3881, -3896, -3910, -3924, -3938, -3951, -3965, -3979, -3992, -4006, -4019, -4032, -4045, -4059, -4071, -4084, -4097, -4110, -4123, -4135, -4148, -4160, -4172, -4184, -4196, -4208, -4220, -4232, -4244, -4255, -4267, -4278, -4290, -4301, -4312, -4323, -4334, -4345, -4356, -4367, -4377, -4388, -4398, -4409, -4419, -4429, -4439, -4449, -4459, -4469, -4479, -4489, -4498, -4508, -4517, -4526, -4535, -4545, -4554, -4563, -4571, -4580, -4589, -4597, -4606, -4614, -4622, -4631, -4639, -4647, -4655, -4663, -4670, -4678, -4686, -4693, -4701, -4708, -4715, -4722, -4729, -4736, -4743, -4750, -4756, -4763, -4769, -4776, -4782, -4788, -4795, -4801, -4807, -4812, -4818, -4824, -4829, -4835, -4840, -4846, -4851, -4856, -4861, -4866, -4871, -4876, -4880, -4885, -4890, -4894, -4898, -4903, -4907, -4911, -4915, -4919, -4922, -4926, -4930, -4933, -4937, -4940, -4943, -4946, -4949, -4952, -4955, -4958, -4961, -4963, -4966, -4968, -4971, -4973, -4975, -4977, -4979, -4981, -4983, -4985, -4986, -4988, -4989, -4991, -4992, -4993, -4994, -4995, -4996, -4997, -4997, -4998, -4999, -4999, -4999, -5000, -5000, -5000, -5000, -5000, -5000, -4999, -4999, -4999, -4998, -4997, -4997, -4996, -4995, -4994, -4993, -4992, -4991, -4989, -4988, -4986, -4985, -4983, -4981, -4979, -4977, -4975, -4973, -4971, -4968, -4966, -4963, -4961, -4958, -4955, -4952, -4949, -4946, -4943, -4940, -4937, -4933, -4930, -4926, -4922, -4919, -4915, -4911, -4907, -4903, -4898, -4894, -4890, -4885, -4880, -4876, -4871, -4866, -4861, -4856, -4851, -4846, -4840, -4835, -4829, -4824, -4818, -4812, -4807, -4801, -4795, -4788, -4782, -4776, -4769, -4763, -4756, -4750, -4743, -4736, -4729, -4722, -4715, -4708, -4701, -4693, -4686, -4678, -4670, -4663, -4655, -4647, -4639, -4631, -4622, -4614, -4606, -4597, -4589, -4580, -4571, -4563, -4554, -4545, -4535, -4526, -4517, -4508, -4498, -4489, -4479, -4469, -4459, -4449, -4439, -4429, -4419, -4409, -4398, -4388, -4377, -4367, -4356, -4345, -4334, -4323, -4312, -4301, -4290, -4278, -4267, -4255, -4244, -4232, -4220, -4208, -4196, -4184, -4172, -4160, -4148, -4135, -4123, -4110, -4097, -4084, -4071, -4059, -4045, -4032, -4019, -4006, -3992, -3979, -3965, -3951, -3938, -3924, -3910, -3896, -3881, -3867, -3853, -3838, -3824, -3809, -3795, -3780, -3765, -3750, -3735, -3720, -3705, -3690, -3675, -3660, -3645, -3630, -3615, -3600, -3585, -3570, -3555, -3540, -3525, -3510, -3495, -3480, -3465, -3450, -3435, -3420, -3405, -3390, -3375, -3360, -3345, -3330, -3315, -3300, -3285, -3270, -3255, -3239, -3224, -3209, -3194, -3179, -3164, -3149, -3134, -3119, -3104, -3089, -3074, -3059, -3044, -3029, -3014, -2999, -2984, -2969, -2954, -2939, -2924, -2909, -2894, -2879, -2864, -2849, -2834, -2819, -2804, -2789, -2774, -2759, -2744, -2729, -2714, -2699, -2684, -2669, -2654, -2639, -2624, -2609, -2594, -2579, -2564, -2549, -2534, -2519, -2504, -2489, -2474, -2459, -2444, -2429, -2414, -2399, -2384, -2369, -2354, -2339, -2324, -2309, -2294, -2279, -2264, -2248, -2233, -2218, -2203, -2188, -2173, -2158, -2143, -2128, -2113, -2098, -2083, -2068, -2053, -2038, -2023, -2008, -1993, -1978, -1963, -1948, -1933, -1918, -1903, -1888, -1873, -1858, -1843, -1828, -1813, -1798, -1783, -1768, -1753, -1738, -1723, -1708, -1693, -1678, -1663, -1648, -1633, -1618, -1603, -1588, -1573, -1558, -1543, -1528, -1513, -1498, -1483, -1468, -1453, -1438, -1423, -1408, -1393, -1378, -1363, -1348, -1333, -1318, -1303, -1288, -1273, -1258, -1243, -1228, -1213, -1198, -1183, -1169, -1154, -1140, -1126, -1111, -1097, -1083, -1069, -1056, -1042, -1028, -1015, -1001, -988, -974, -961, -948, -935, -922, -909, -896, -884, -871, -859, -846, -834, -822, -810, -798, -786, -774, -762, -750, -739, -727, -716, -705, -693, -682, -671, -660, -649, -639, -628, -617, -607, -596, -586, -576, -566, -556, -546, -536, -526, -516, -507, -497, -488, -478, -469, -460, -451, -442, -433, -424, -416, -407, -398, -390, -382, -373, -365, -357, -349, -341, -333, -326, -318, -311, -303, -296, -289, -281, -274, -267, -260, -254, -247, -240, -234, -227, -221, -215, -208, -202, -196, -191, -185, -179, -173, -168, -162, -157, -152, -146, -141, -136, -131, -127, -122, -117, -113, -108, -104, -100, -95, -91, -87, -83, -80, -76, -72, -69, -65, -62, -58, -55, -52, -49, -46, -43, -41, -38, -35, -33, -30, -28, -26, -24, -22, -20, -18, -16, -15, -13, -12, -10, -9, -8, -6, -5, -5, -4, -3, -2, -2, -1, -1, 0, 0, 0, 0};
  int dxl_goal_velocity[] = {0, -5, -9, -14, -18, -23, -27, -32, -36, -41, -45, -50, -54, -59, -63, -68, -72, -77, -81, -86, -90, -95, -99, -104, -108, -113, -117, -122, -126, -131, -135, -140, -144, -149, -153, -158, -162, -167, -171, -176, -180, -185, -189, -194, -198, -203, -207, -212, -216, -221, -225, -230, -234, -239, -243, -248, -252, -257, -261, -266, -270, -275, -279, -284, -288, -293, -297, -302, -306, -311, -315, -320, -324, -329, -333, -338, -342, -347, -351, -356, -360, -365, -369, -374, -378, -383, -387, -392, -396, -401, -405, -410, -414, -419, -423, -428, -432, -437, -441, -446, -450, -455, -459, -464, -468, -473, -477, -482, -486, -491, -495, -500, -505, -509, -514, -518, -523, -527, -532, -536, -541, -545, -550, -554, -559, -563, -568, -572, -577, -581, -586, -590, -595, -599, -604, -608, -613, -617, -622, -626, -631, -635, -640, -644, -649, -653, -658, -662, -667, -671, -676, -680, -685, -689, -694, -698, -703, -707, -712, -716, -721, -725, -730, -734, -739, -743, -748, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -750, -745, -741, -736, -732, -727, -723, -718, -714, -709, -705, -700, -696, -691, -687, -682, -678, -673, -669, -664, -660, -655, -651, -646, -642, -637, -633, -628, -624, -619, -615, -610, -606, -601, -597, -592, -588, -583, -579, -574, -570, -565, -561, -556, -552, -547, -543, -538, -534, -529, -525, -520, -516, -511, -507, -502, -498, -493, -489, -484, -480, -475, -471, -466, -462, -457, -453, -448, -444, -439, -435, -430, -426, -421, -417, -412, -408, -403, -399, -394, -390, -385, -381, -376, -372, -367, -363, -358, -354, -349, -345, -340, -336, -331, -327, -322, -318, -313, -309, -304, -300, -295, -291, -286, -282, -277, -273, -268, -264, -259, -255, -250, -245, -241, -236, -232, -227, -223, -218, -214, -209, -205, -200, -196, -191, -187, -182, -178, -173, -169, -164, -160, -155, -151, -146, -142, -137, -133, -128, -124, -119, -115, -110, -106, -101, -97, -92, -88, -83, -79, -74, -70, -65, -61, -56, -52, -47, -43, -38, -34, -29, -25, -20, -16, -11, -7, -2, 2, 7, 11, 16, 20, 25, 29, 34, 38, 43, 47, 52, 56, 61, 65, 70, 74, 79, 83, 88, 92, 97, 101, 106, 110, 115, 119, 124, 128, 133, 137, 142, 146, 151, 155, 160, 164, 169, 173, 178, 182, 187, 191, 196, 200, 205, 209, 214, 218, 223, 227, 232, 236, 241, 245, 250, 255, 259, 264, 268, 273, 277, 282, 286, 291, 295, 300, 304, 309, 313, 318, 322, 327, 331, 336, 340, 345, 349, 354, 358, 363, 367, 372, 376, 381, 385, 390, 394, 399, 403, 408, 412, 417, 421, 426, 430, 435, 439, 444, 448, 453, 457, 462, 466, 471, 475, 480, 484, 489, 493, 498, 502, 507, 511, 516, 520, 525, 529, 534, 538, 543, 547, 552, 556, 561, 565, 570, 574, 579, 583, 588, 592, 597, 601, 606, 610, 615, 619, 624, 628, 633, 637, 642, 646, 651, 655, 660, 664, 669, 673, 678, 682, 687, 691, 696, 700, 705, 709, 714, 718, 723, 727, 732, 736, 741, 745, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 750, 748, 743, 739, 734, 730, 725, 721, 716, 712, 707, 703, 698, 694, 689, 685, 680, 676, 671, 667, 662, 658, 653, 649, 644, 640, 635, 631, 626, 622, 617, 613, 608, 604, 599, 595, 590, 586, 581, 577, 572, 568, 563, 559, 554, 550, 545, 541, 536, 532, 527, 523, 518, 514, 509, 505, 500, 495, 491, 486, 482, 477, 473, 468, 464, 459, 455, 450, 446, 441, 437, 432, 428, 423, 419, 414, 410, 405, 401, 396, 392, 387, 383, 378, 374, 369, 365, 360, 356, 351, 347, 342, 338, 333, 329, 324, 320, 315, 311, 306, 302, 297, 293, 288, 284, 279, 275, 270, 266, 261, 257, 252, 248, 243, 239, 234, 230, 225, 221, 216, 212, 207, 203, 198, 194, 189, 185, 180, 176, 171, 167, 162, 158, 153, 149, 144, 140, 135, 131, 126, 122, 117, 113, 108, 104, 99, 95, 90, 86, 81, 77, 72, 68, 63, 59, 54, 50, 45, 41, 36, 32, 27, 23, 18, 14, 9, 5, 0};
  int goal_position_size = sizeof(dxl_goal_position) / sizeof(int);

  uint8_t dxl_error = 0;                            // DYNAMIXEL error
  uint8_t param_goal_position[4];
  int32_t dxl1_present_position = 0, dxl2_present_position = 0;                         // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable DYNAMIXEL#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("DYNAMIXEL#%d has been successfully connected \n", DXL1_ID);
  }

  // Enable DYNAMIXEL#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    printf("DYNAMIXEL#%d has been successfully connected \n", DXL2_ID);
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    for (index = 0; index < goal_position_size; index++)
    {
      // Allocate goal position value into byte array
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[index]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity[index]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity[index]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity[index]));

      // Add Dynamixel#1 goal position value to the Syncwrite storage
      dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return 0;
      }

      // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return 0;
      }

      // Syncwrite goal position
      dxl_comm_result = groupSyncWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

      // Clear syncwrite parameter storage
      groupSyncWrite.clearParam();

      // Allocate goal position value into byte array
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

      // Add DYNAMIXEL#1 goal position value to the FastSyncwrite storage
      dxl_addparam_result = groupFastSyncWrite.addParam(DXL1_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupFastSyncWrite addparam failed", DXL1_ID);
        return 0;
      }

      // Add DYNAMIXEL#2 goal position value to the FastSyncwrite parameter storage
      dxl_addparam_result = groupFastSyncWrite.addParam(DXL2_ID, param_goal_position);
      if (dxl_addparam_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupFastSyncWrite addparam failed", DXL2_ID);
        return 0;
      }

      // FastSyncWrite write goal position and present position
      dxl_comm_result = groupFastSyncWrite.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (groupFastSyncWrite.getError(DXL1_ID, &dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL1_ID, packetHandler->getRxPacketError(dxl_error));
      }
      else if (groupFastSyncWrite.getError(DXL2_ID, &dxl_error))
      {
        printf("[ID:%03d] %s\n", DXL2_ID, packetHandler->getRxPacketError(dxl_error));
      }

      // Check if groupFastSyncWrite data of DYNAMIXEL#1 is available
      dxl_getdata_result = groupFastSyncWrite.isAvailable(DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupFastSyncWrite getdata failed", DXL1_ID);
        return 0;
      }

      // Check if groupFastSyncWrite data of DYNAMIXEL#2 is available
      dxl_getdata_result = groupFastSyncWrite.isAvailable(DXL2_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
      if (dxl_getdata_result != true)
      {
        fprintf(stderr, "[ID:%03d] groupFastSyncWrite getdata failed", DXL2_ID);
        return 0;
      }

      // Get DYNAMIXEL#1 present position value
      dxl1_present_position = groupFastSyncWrite.getData(DXL1_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

      // Get DYNAMIXEL#2 present position value
      dxl2_present_position = groupFastSyncWrite.getData(DXL2_ID, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

      // Clear syncwrite parameter storage
      groupFastSyncWrite.clearParam();

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position[index], dxl2_present_position);
    
      std::chrono::milliseconds delay(10);
      std::this_thread::sleep_for(delay);
    }
  }

  // Disable DYNAMIXEL#1 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Disable DYNAMIXEL#2 Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
