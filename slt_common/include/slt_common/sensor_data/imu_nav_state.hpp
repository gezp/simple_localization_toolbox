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

struct ImuNavState
{
  double time;
  // imu pose and linear velocity in world frame
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  // imu bias in body frame
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  // gravity in world frame
  Eigen::Vector3d gravity;
};

}  // namespace slt_common
