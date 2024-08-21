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

////////////////////////////////////////////////////////////////////////////////
/// @file The file for Dynamixel Fast Sync Read
/// @author Honghyun Kim
////////////////////////////////////////////////////////////////////////////////

#ifndef DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTSYNCWRITE_H
#define DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTSYNCWRITE_H


#include "group_fast_sync_read.h"

namespace dynamixel
{

class WINDECLSPEC GroupFastSyncWrite : public GroupFastSyncRead
{
protected:
    void makeParam();

public:
    GroupFastSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);
    ~GroupFastSyncWrite() { clearParam(); }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that adds id, start_address, data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data Data for write
  /// @return false
  /// @return   when the ID exists already in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    addParam    (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that removes id from the Sync Write list
  /// @param id Dynamixel ID
  ////////////////////////////////////////////////////////////////////////////////
  void    removeParam (uint8_t id);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that changes the data for write in id -> start_address -> data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data for replacement
  /// @return false
  /// @return   when the ID doesn't exist in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    changeParam (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that clears the Sync Write list
  ////////////////////////////////////////////////////////////////////////////////
  void    clearParam();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that transmits the Sync Write instruction packet which might be constructed by GroupSyncWrite::addParam function
  /// @return COMM_NOT_AVAILABLE
  /// @return   when the list for Sync Write is empty
  /// @return or the other communication results which come from PacketHandler::syncWriteTxOnly
  ////////////////////////////////////////////////////////////////////////////////
    int txPacket();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that transmits and receives the packet which might be come from the Dynamixel
  /// @return COMM_NOT_AVAILABLE
  /// @return   when the protocol1.0 has been used
  /// @return COMM_RX_FAIL
  /// @return   when there is no packet recieved
  /// @return COMM_SUCCESS
  /// @return   when there is packet recieved
  /// @return or the other communication results which come from GroupBulkRead::txPacket or GroupBulkRead::rxPacket
  ////////////////////////////////////////////////////////////////////////////////
  int     txRxPacket();
};

}


#endif // DYNAMIXEL_SDK_INCLUDE_DYNAMIXEL_SDK_GROUPFASTSYNCWRITE_H
