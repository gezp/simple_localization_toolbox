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

#include "slt_common/cloud_filter/voxel_filter.hpp"

namespace slt_common
{

VoxelFilter::VoxelFilter(const YAML::Node & node)
{
  leaf_size_.x() = node["leaf_size"][0].as<double>();
  leaf_size_.y() = node["leaf_size"][1].as<double>();
  leaf_size_.z() = node["leaf_size"][2].as<double>();
  enable_ = node["enable"].as<bool>();
  voxel_filter_.setLeafSize(leaf_size_.x(), leaf_size_.y(), leaf_size_.z());
}

VoxelFilter::VoxelFilter(const Eigen::Vector3d & leaf_size)
{
  leaf_size_ = leaf_size;
  voxel_filter_.setLeafSize(leaf_size_.x(), leaf_size_.y(), leaf_size_.z());
}

VoxelFilter::PointCloudPtr VoxelFilter::apply(const VoxelFilter::PointCloudPtr & input)
{
  PointCloudPtr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (enable_) {
    voxel_filter_.setInputCloud(input);
    voxel_filter_.filter(*output_cloud);
  } else {
    pcl::copyPointCloud(*input, *output_cloud);
  }
  return output_cloud;
}

void VoxelFilter::print_info()
{
  std::cout << "[voxel_filter] "
            << "leaf size:" << voxel_filter_.getLeafSize().transpose() << std::endl;
}

}  // namespace slt_common
