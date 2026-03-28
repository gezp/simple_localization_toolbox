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
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "slt_common/loam/loam_feature_extraction.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/tic_toc.hpp"

namespace slt_common
{

class LoamRegistration
{
  using PointType = pcl::PointXYZL;
  using PointCloudType = pcl::PointCloud<pcl::PointXYZL>;
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZL>::Ptr;

public:
  struct LoamEdgeCorrespondence
  {
    Eigen::Vector3d current_point;
    Eigen::Vector3d last_p_j;
    Eigen::Vector3d last_p_l;
  };

  struct LoamPlanarCorrespondence
  {
    Eigen::Vector3d current_point;
    Eigen::Vector3d last_p_j;
    Eigen::Vector3d last_p_l;
    Eigen::Vector3d last_p_m;
  };
  LoamRegistration() = default;
  ~LoamRegistration() = default;
  explicit LoamRegistration(const YAML::Node & config);
  bool set_target(const LoamFeature & target);
  bool match(const LoamFeature & input, const Eigen::Matrix4d & initial_pose);
  Eigen::Matrix4d get_final_pose();

private:
  std::vector<LoamEdgeCorrespondence> find_all_edge_correspondence(
    const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose);
  std::vector<LoamPlanarCorrespondence> find_all_planar_correspondence(
    const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose);

private:
  // correspondence search config
  int num_nearby_ring_{2};
  double sqr_distance_threshold_{25};
  // optimization config
  int num_optimization_{2};
  bool use_analytic_derivatives_{false};
  double ceres_loss_{0.1};
  int max_num_iterations_{4};
  int min_num_residual_blocks_{10};
  bool minimizer_progress_to_stdout_{false};
  bool debug_{false};
  // data
  LoamFeature last_feature_;
  pcl::KdTreeFLANN<PointType>::Ptr last_edge_kd_tree_;
  pcl::KdTreeFLANN<PointType>::Ptr last_planar_kd_tree_;
  // ceres parameter (qx,qy,qz,qw,tx,ty,tz)
  double ceres_parameter_[7];
  // debug
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};
}  // namespace slt_common
