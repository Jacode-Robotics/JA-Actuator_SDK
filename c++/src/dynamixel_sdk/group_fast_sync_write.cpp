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

/* Author: Honghyun Kim */

#include <algorithm>

#if defined(__linux__)
#include "group_fast_sync_write.h"
#elif defined(__APPLE__)
#include "group_fast_sync_write.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "group_fast_sync_write.h"
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__)
#include "../../include/dynamixel_sdk/group_fast_sync_write.h"
#endif

using namespace dynamixel;

GroupFastSyncWrite::GroupFastSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : GroupFastSyncRead(port, ph, start_address, data_length)
{
    clearParam();
}

void GroupFastSyncWrite::makeParam()
{
  if (id_list_.size() == 0) return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  param_ = new uint8_t[id_list_.size() * (1 + data_length_)]; // ID(1) + DATA(data_length)

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    if (data_list_[id] == 0)
      return;

    param_[idx++] = id;
    for (int c = 0; c < data_length_; c++)
      param_[idx++] = (data_list_[id])[c];
  }
}

bool GroupFastSyncWrite::addParam(uint8_t id, uint8_t *data)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];
  error_list_[id] = new uint8_t[1];

  is_param_changed_   = true;
  return true;
}

void GroupFastSyncWrite::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  delete[] data_list_[id];
  delete[] error_list_[id];
  data_list_.erase(id);
  error_list_.erase(id);

  is_param_changed_   = true;
}

bool GroupFastSyncWrite::changeParam(uint8_t id, uint8_t *data)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return false;

  delete[] data_list_[id];
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void GroupFastSyncWrite::clearParam()
{
  if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    delete[] data_list_[id_list_[i]];
    delete[] error_list_[id_list_[i]];
  }

  id_list_.clear();
  data_list_.clear();
  error_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

int GroupFastSyncWrite::txPacket()
{
    if ((1.0 == ph_->getProtocolVersion()) || (id_list_.empty()))
        return COMM_NOT_AVAILABLE;

    if ((true == is_param_changed_) || (0 == param_))
        makeParam();

    return ph_->fastSyncWriteTx(port_, start_address_, data_length_, param_, (uint16_t)id_list_.size() * (1 + data_length_));
}

int GroupFastSyncWrite::txRxPacket()
{
    if (1.0 == ph_->getProtocolVersion())
        return COMM_NOT_AVAILABLE;

    int result = txPacket();
    if (COMM_SUCCESS != result)
        return result;
    return rxPacket();
}
