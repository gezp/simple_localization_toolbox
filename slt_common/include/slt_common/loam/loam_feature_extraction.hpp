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

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "slt_common/sensor_data/lidar_data.hpp"

namespace slt_common
{

struct LoamFeature
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZL>::Ptr;
  PointCloudPtr corner_sharp;
  PointCloudPtr corner_less_sharp;
  PointCloudPtr surface_flat;
  PointCloudPtr surface_less_flat;
};

class LoamFeatureExtraction
{
  using PointType = pcl::PointXYZL;
  using PointCloudType = pcl::PointCloud<pcl::PointXYZL>;
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZL>::Ptr;

public:
  LoamFeatureExtraction() = default;
  ~LoamFeatureExtraction() = default;
  explicit LoamFeatureExtraction(const YAML::Node & config);
  bool extract(const pcl::PointCloud<PointXYZIRT>::Ptr & point_cloud, LoamFeature & feature);
  void print_feature_info(const LoamFeature & feature);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_feature_point_cloud(const LoamFeature & feature);
  LoamFeature transform_feature(const LoamFeature & input, const Eigen::Matrix4d & pose);

private:
  double get_point_curvature(const PointCloudType & ring_point_cloud, int idx);
  void mark_neighbor_points(
    const PointCloudType & ring_point_cloud, std::vector<bool> & mark, int idx);

private:
  // config
  int num_rings_{64};
  int num_regions_{6};
  int min_point_size_in_ring_{131};
  // for get_point_curvature
  int curvature_padding_{5};
  // for mark_neighbor_points
  int neighbor_padding_{5};
  double neighbor_distance_threshold_{0.05};
  // curvature threshold
  double corner_curvature_threshold_{0.1};
  double surface_curvature_threshold_{0.1};
  // max num in each region of each ring
  int max_num_corner_sharp_{2};
  int max_num_corner_less_sharp_{20};
  int max_num_surface_flat_{2};
  // voxel filter leaf size for surface_less_flat
  double voxel_filter_leaf_size_{0.2};
  // for debug
  std::vector<int> corner_sharp_rgb_{0, 0, 255};
  std::vector<int> corner_less_sharp_rgb_{0, 0, 255};
  std::vector<int> surface_flat_rgb_{0, 0, 255};
  std::vector<int> surface_less_flat_rgb_{0, 0, 255};
  bool debug_{false};
};
}  // namespace slt_common
