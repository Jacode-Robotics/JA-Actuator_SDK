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

#include <gtest/gtest.h>

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
#include "profile.h"

// Control table address
#define ADDR_PRO_TORQUE_ENABLE          512                 // Control table address is different in DYNAMIXEL model
#define ADDR_PRO_GOAL_POSITION          564
#define ADDR_PRO_DRIVE_MODE             10

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL

// Default setting
const uint8_t JA_ID[] =                 {1, 2};
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
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

  // return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

TEST(TrajTest, Planning)
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
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
  dynamixel::GroupFastSyncWrite groupFastSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

  // Create a PF_Handle object and initialize it
  int32_t initial_position = 0;
  PF_Handle profile(initial_position);

  int dxl_comm_result = COMM_TX_FAIL;               // Communication result
  bool dxl_addparam_result = false;                 // addParam result
  bool dxl_getdata_result = false;                  // GetParam result

  uint8_t dxl_error = 0;                            // DYNAMIXEL error
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
    // return 0;
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
    // return 0;
  }

  // Allocate goal position value into byte array
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(0));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(0));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(0));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(0));

  // Homing
  for (size_t i = 0; i < sizeof(JA_ID); i++)
  {
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

    // Add goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(JA_ID[i], param_goal_position);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", JA_ID[i]);
      // return 0;
    }
  }

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();    
  std::chrono::milliseconds delay(3000);
  std::this_thread::sleep_for(delay);

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
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Set a new goal position
    profile.NewGoalPos(initial_position, 5000);

    while (!profile.ExecutionPos())
    {
      // Allocate goal position value into byte array
      param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(profile.trajectory_pos));
      param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(profile.trajectory_pos));
      param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(profile.trajectory_pos));
      param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(profile.trajectory_pos));

      // Add goal position value to the FastSyncwrite storage
      for (size_t i = 0; i < sizeof(JA_ID); i++)
      {
        dxl_addparam_result = groupFastSyncWrite.addParam(JA_ID[i], param_goal_position);
        if (dxl_addparam_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupFastSyncWrite addparam failed", JA_ID[i]);
          // return 0;
        }
      }

      // FastSyncWrite write goal position and present position
      dxl_comm_result = groupFastSyncWrite.txRxPacket();
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }

      for (size_t i = 0; i < sizeof(JA_ID); i++)
      {
        if (groupFastSyncWrite.getError(JA_ID[i], &dxl_error))
        {
          printf("[ID:%03d] %s\n", JA_ID[i], packetHandler->getRxPacketError(dxl_error));
        }

        // Check if groupFastSyncWrite data of DYNAMIXEL#1 is available
        dxl_getdata_result = groupFastSyncWrite.isAvailable(JA_ID[i], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);
        if (dxl_getdata_result != true)
        {
          fprintf(stderr, "[ID:%03d] groupFastSyncWrite getdata failed", JA_ID[i]);
          // return 0;
        }

        // Get DYNAMIXEL#1 present position value
        present_position = groupFastSyncWrite.getData(JA_ID[i], ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t", JA_ID[i], profile.trajectory_pos, present_position);
      }
      printf("\n");

      // Clear syncwrite parameter storage
      groupFastSyncWrite.clearParam();
    
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

    // Enable Profile
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, JA_ID[i], ADDR_PRO_DRIVE_MODE, PROFILE_ENABLE, &dxl_error);
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

  // return 0;
}
