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

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Sync Read and Sync Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
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

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          512                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          564
#define ADDR_PRO_PRESENT_POSITION       580
#define ADDR_PRO_DRIVE_MODE             10

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
const uint8_t JA_ID[] =                 {1};
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      5000              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#define PROFILE_ENABLE                  0x0                  // Value for enable trajectory profile
#define PROFILE_DISABLE                 0x02                 // Value for disable trajectory profile
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

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Initialize Groupsyncread instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

  size_t index = 0;
  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result
  int dxl_goal_position[] = {0, 1, 5, 10, 18, 28, 41, 56, 73, 92, 114, 137, 164, 192, 223, 256, 291, 328, 368, 410, 455, 501, 550, 601, 655, 710, 768, 828, 891, 956, 1023, 1092, 1164, 1237, 1313, 1388, 1464, 1539, 1614, 1690, 1765, 1840, 1916, 1991, 2067, 2142, 2217, 2293, 2368, 2443, 2519, 2594, 2670, 2745, 2820, 2896, 2971, 3046, 3122, 3197, 3273, 3348, 3423, 3499, 3574, 3649, 3725, 3800, 3872, 3943, 4011, 4077, 4141, 4202, 4261, 4318, 4372, 4425, 4475, 4522, 4568, 4611, 4652, 4691, 4727, 4761, 4793, 4822, 4850, 4875, 4897, 4918, 4936, 4952, 4966, 4977, 4986, 4993, 4997, 5000, 5000, 4997, 4993, 4986, 4977, 4966, 4952, 4936, 4918, 4897, 4875, 4850, 4822, 4793, 4761, 4727, 4691, 4652, 4611, 4568, 4522, 4475, 4425, 4372, 4318, 4261, 4202, 4141, 4077, 4011, 3943, 3872, 3800, 3725, 3649, 3574, 3499, 3423, 3348, 3273, 3197, 3122, 3046, 2971, 2896, 2820, 2745, 2670, 2594, 2519, 2443, 2368, 2293, 2217, 2142, 2067, 1991, 1916, 1840, 1765, 1690, 1614, 1539, 1464, 1388, 1313, 1237, 1164, 1092, 1023, 956, 891, 828, 768, 710, 655, 601, 550, 501, 455, 410, 368, 328, 291, 256, 223, 192, 164, 137, 114, 92, 73, 56, 41, 28, 18, 10, 5, 1, 0};
  uint8_t dxl_error = 0;                            // Dynamixel error
  uint8_t param_goal_position[4];
  int32_t present_position = 0;                         // Present position

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

  for (size_t i = 0; i < sizeof(JA_ID); i++)
  {
    // Disable Profile
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JA_ID[i], ADDR_PRO_DRIVE_MODE, PROFILE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }

    // Enable Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JA_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
      printf("Dynamixel#%d has been successfully connected \n", JA_ID[i]);
    }

    // Add parameter storage for Dynamixel#1 present position value
    dxl_addparam_result = groupSyncRead.addParam(JA_ID[i]);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", JA_ID[i]);
      return 0;
    }
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    for (index = 0; index < sizeof(dxl_goal_position)/sizeof(int); index++)
    {
      // Allocate goal position value into byte array
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index]));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index]));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index]));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]));

      // Syncread present position
      dxl_comm_result = groupSyncRead.txRxPacket();

      for (size_t i = 0; i < sizeof(JA_ID); i++)
      {
        if (dxl_comm_result != COMM_SUCCESS)
        {
          printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (groupSyncRead.getError(JA_ID[i], &dxl_error))
        {
          printf("[ID:%03d] %s\n", JA_ID[i], packetHandler->getRxPacketError(dxl_error));
        }

        // Check if groupsyncread data is available
        dxl_getdata_result = groupSyncRead.isAvailable(JA_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if (dxl_getdata_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", JA_ID[i]);
          return 0;
        }

        // Get present position value
        present_position = groupSyncRead.getData(JA_ID[i], ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", JA_ID[i], dxl_goal_position[index], present_position);

        // Add goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite.addParam(JA_ID[i], param_goal_position);
        if (dxl_addparam_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", JA_ID[i]);
          return 0;
        }
      }

      // Syncwrite goal position
      dxl_comm_result = groupSyncWrite.txPacket();
      if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

      // Clear syncwrite parameter storage
      groupSyncWrite.clearParam();

      std::chrono::milliseconds delay(10);
      std::this_thread::sleep_for(delay);
    }
  }

  for (size_t i = 0; i < sizeof(JA_ID); i++)
  {
    // Disable Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JA_ID[i], ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
  }

  // Close port
  portHandler->closePort();

  return 0;
}
