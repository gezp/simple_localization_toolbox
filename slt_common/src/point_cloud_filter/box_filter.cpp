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

#include "slt_common/point_cloud_filter/box_filter.hpp"

#include <iostream>
#include <vector>

namespace slt_common
{
BoxFilter::BoxFilter(const YAML::Node & node)
{
  origin_ = Eigen::Vector3d::Zero();
  size_.resize(6);
  for (size_t i = 0; i < size_.size(); i++) {
    size_.at(i) = node["box_filter_size"][i].as<double>();
  }
  calculate_edge();
}

BoxFilter::BoxFilter(const Eigen::Vector3d & origin, const std::vector<double> & size)
{
  origin_ = origin;
  size_ = size;
  calculate_edge();
}

void BoxFilter::set_size(const std::vector<double> & size)
{
  size_ = size;
  calculate_edge();
}

void BoxFilter::set_origin(const Eigen::Vector3d & origin)
{
  origin_ = origin;
  calculate_edge();
}

void BoxFilter::print_info()
{
  std::cout << "[box_filter] "
            << "min_x: " << min_point_.x() << ", "
            << "max_x: " << max_point_.x() << ", "
            << "min_y: " << min_point_.y() << ", "
            << "max_y: " << max_point_.y() << ", "
            << "min_z: " << min_point_.z() << ", "
            << "max_z: " << max_point_.z() << std::endl;
}

BoxFilter::PointCloudPtr BoxFilter::apply(const BoxFilter::PointCloudPtr & input)
{
  PointCloudPtr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_box_filter_.setMin(Eigen::Vector4f(min_point_.x(), min_point_.y(), min_point_.z(), 1.0));
  pcl_box_filter_.setMax(Eigen::Vector4f(max_point_.x(), max_point_.y(), max_point_.z(), 1.0));
  pcl_box_filter_.setInputCloud(input);
  pcl_box_filter_.filter(*output_cloud);
  return output_cloud;
}

const Eigen::Vector3d & BoxFilter::get_min_point()
{
  return min_point_;
}

const Eigen::Vector3d & BoxFilter::get_max_point()
{
  return max_point_;
}

void BoxFilter::calculate_edge()
{
  for (size_t i = 0; i < 3; ++i) {
    min_point_(i) = origin_(i) + size_.at(2 * i);
    max_point_(i) = origin_(i) + size_.at(2 * i + 1);
  }
}

}  // namespace slt_common
