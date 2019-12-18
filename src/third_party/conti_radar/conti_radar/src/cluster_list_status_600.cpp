/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cluster_list_status_600.h"

#include "glog/logging.h"

#include "byte.h"
#include "canbus_consts.h"

ClusterListStatus600::ClusterListStatus600() {}
const uint32_t ClusterListStatus600::ID = 0x600;

void ClusterListStatus600::Parse(const std::uint8_t* bytes, int32_t length,
                                 ContiRadar& conti_radar) const {
  //auto status = conti_radar->mutable_cluster_list_status();
  conti_radar.cluster_list_status.near=near(bytes, length);
  conti_radar.cluster_list_status.far=far(bytes, length);
  conti_radar.cluster_list_status.meas_counter=meas_counter(bytes, length);
  conti_radar.cluster_list_status.interface_version=interface_version(bytes, length);

  auto counter = conti_radar.cluster_list_status.near +  conti_radar.cluster_list_status.far;
  //conti_radar->mutable_contiobs()->Reserve(counter);  //??? 目前不清楚
}

int ClusterListStatus600::near(const std::uint8_t* bytes,
                               int32_t length) const {
  Byte t0(bytes);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ClusterListStatus600::far(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int ClusterListStatus600::meas_counter(const std::uint8_t* bytes,
                                       int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 3);
  uint32_t t = t0.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x;
  return ret;
}

int ClusterListStatus600::interface_version(const std::uint8_t* bytes,
                                            int32_t length) const {
  Byte t0(bytes + 4);
  int32_t x = t0.get_byte(4, 4);

  int ret = x;
  return ret;
}
