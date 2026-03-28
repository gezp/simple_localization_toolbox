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
#include <pcl/filters/voxel_grid.h>
#include <yaml-cpp/yaml.h>

namespace slt_common
{

class VoxelFilter
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  explicit VoxelFilter(const YAML::Node & node);
  explicit VoxelFilter(const Eigen::Vector3d & leaf_size);
  PointCloudPtr apply(const PointCloudPtr & input);
  void print_info();

private:
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  Eigen::Vector3d leaf_size_;
  bool enable_{true};
};

}  // namespace slt_common
