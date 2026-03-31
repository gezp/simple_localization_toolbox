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

#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>

#include <vector>

#include "slt_common/point_cloud_registration/point_cloud_registration_interface.hpp"

namespace slt_common
{

class IcpSvd : public PointCloudRegistrationInterface
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  explicit IcpSvd(const YAML::Node & node);
  IcpSvd(float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter);

  bool set_target(const PointCloudPtr & target) override;
  bool match(const PointCloudPtr & input, const Eigen::Matrix4d & initial_pose) override;
  Eigen::Matrix4d get_final_pose() override;
  double get_fitness_score() override;
  void print_info() override;

private:
  bool set_param(float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter);
  size_t get_correspondence(
    const PointCloudPtr & input_source, std::vector<Eigen::Vector3f> & xs,
    std::vector<Eigen::Vector3f> & ys);
  void get_transform(
    const std::vector<Eigen::Vector3f> & xs, const std::vector<Eigen::Vector3f> & ys,
    Eigen::Matrix4f & transformation);
  bool is_significant(const Eigen::Matrix4f & transformation, const float trans_eps);

private:
  float max_corr_dist_;
  float trans_eps_;
  float euc_fitness_eps_;
  int max_iter_;

  PointCloudPtr input_target_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr input_target_kdtree_;
  PointCloudPtr input_source_;

  Eigen::Matrix4f transformation_;
  Eigen::Matrix4d final_pose_;
};

}  // namespace slt_common
