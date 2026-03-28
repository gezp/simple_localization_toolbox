// Copyright 2023 Gezp (https://github.com/gezp).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Dense>

namespace slt_common
{

// https://github.com/gaoxiang12/slam_in_autonomous_driving/blob/master/src/common/gnss.h
/// GNSS状态位信息, 通常由GNSS厂商提供，这里使用千寻提供的状态位
enum class GnssStatus
{
  FIXED_SOLUTION = 4,         // 固定解（cm级）
  FLOAT_SOLUTION = 3,         // 浮点解（cm到dm之间）
  PSEUDO_SOLUTION = 2,        // 伪距差分解（dm级）
  SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
  NOT_EXIST = 0,              // GPS无信号
  OTHER = -1,                 // 其他
};

struct GnssData
{
  double time = 0.0;
  GnssStatus status = GnssStatus::NOT_EXIST;
  // LLA of antenna (degree, degree, m)
  double longitude = 0.0;
  double latitude = 0.0;
  double altitude = 0.0;
  // position of antenna in ENU (m)
  Eigen::Vector3d antenna_position;
  bool antenna_position_valid = false;
  // heading of dual antenna in ENU (rad)
  double dual_antenna_heading = 0.0;
  bool dual_antenna_heading_valid = false;
};
}  // namespace slt_common
