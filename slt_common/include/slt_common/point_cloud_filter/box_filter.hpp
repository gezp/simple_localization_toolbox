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
#include <pcl/filters/crop_box.h>
#include <yaml-cpp/yaml.h>
#include <vector>

namespace slt_common
{
class BoxFilter
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  explicit BoxFilter(const YAML::Node & node);
  BoxFilter(const Eigen::Vector3d & origin, const std::vector<double> & size);

  void set_size(const std::vector<double> & size);
  void set_origin(const Eigen::Vector3d & origin);
  void print_info();
  PointCloudPtr apply(const PointCloudPtr & input);
  const Eigen::Vector3d & get_min_point();
  const Eigen::Vector3d & get_max_point();

private:
  void calculate_edge();

private:
  pcl::CropBox<pcl::PointXYZ> pcl_box_filter_;
  Eigen::Vector3d origin_;
  std::vector<double> size_;
  Eigen::Vector3d min_point_;
  Eigen::Vector3d max_point_;
};
}  // namespace slt_common
