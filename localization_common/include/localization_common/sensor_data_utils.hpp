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

#include <Eigen/Geometry>

#include "localization_common/sensor_data/twist_data.hpp"
#include "localization_common/sensor_data/imu_data.hpp"
#include "localization_common/sensor_data/odom_data.hpp"
#include "localization_common/sensor_data/pose_data.hpp"

namespace localization_common
{

// Twist b = Tba * Twist a
TwistData transform_twist(const TwistData & twist_a, const Eigen::Matrix4d & T_ba);

// Odom b = Tba * Odom a
OdomData transform_odom(const OdomData & odom_a, const Eigen::Matrix4d & T_ba);

Eigen::Vector3d interpolate_xyz(
  const Eigen::Vector3d & data1, const Eigen::Vector3d & data2, double coeff);

Eigen::Quaterniond interpolate_rotation(
  const Eigen::Quaterniond & data1, const Eigen::Quaterniond & data2, double coeff);

Eigen::Matrix3d interpolate_rotation(
  const Eigen::Matrix3d & data1, const Eigen::Matrix3d & data2, double coeff);

Eigen::Matrix4d interpolate_pose(
  const Eigen::Matrix4d & data1, const Eigen::Matrix4d & data2, double coeff);

ImuData interpolate_imu(const ImuData & data1, const ImuData & data2, double time);

OdomData interpolate_odom(const OdomData & data1, const OdomData & data2, double time);

TwistData get_twist_from_odom(const OdomData & odom);

TwistData estimate_twist_by_pose(
  const Eigen::Matrix4d & pose1, const Eigen::Matrix4d & pose2, double t1, double t2);

TwistData estimate_twist_by_pose(const PoseData & pose1, const PoseData & pose2);

}  // namespace localization_common
