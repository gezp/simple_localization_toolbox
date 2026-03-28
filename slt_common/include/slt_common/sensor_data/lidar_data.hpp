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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace slt_common
{

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

template<typename PointT>
struct LidarData
{
  // measurement time
  double time = 0.0;
  typename pcl::PointCloud<PointT>::Ptr point_cloud = nullptr;
};

}  // namespace slt_common

POINT_CLOUD_REGISTER_POINT_STRUCT(
  slt_common::PointXYZIRT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, ring, ring)(
    double, time, time))
