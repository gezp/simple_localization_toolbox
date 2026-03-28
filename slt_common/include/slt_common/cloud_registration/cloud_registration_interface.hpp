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
#include <Eigen/Dense>

namespace slt_common
{
class CloudRegistrationInterface
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  virtual ~CloudRegistrationInterface() = default;
  virtual bool set_target(const PointCloudPtr & target) = 0;
  virtual bool match(const PointCloudPtr & input, const Eigen::Matrix4d & initial_pose) = 0;
  virtual Eigen::Matrix4d get_final_pose() = 0;
  virtual double get_fitness_score() = 0;
  virtual void print_info() = 0;
};
}  // namespace slt_common
