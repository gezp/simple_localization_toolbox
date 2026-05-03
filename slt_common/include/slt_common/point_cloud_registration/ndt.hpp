// Copyright 2026 Gezp (https://github.com/gezp).
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
#include <yaml-cpp/yaml.h>

#include <memory>

#include "slt_common/point_cloud_registration/ndt/normal_distributions_transform.hpp"
#include "slt_common/point_cloud_registration/point_cloud_registration_interface.hpp"

namespace slt_common
{

// ============================================================================
// Ndt - Public API implementing PointCloudRegistrationInterface
// ============================================================================

class Ndt : public PointCloudRegistrationInterface
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  explicit Ndt(const YAML::Node & node);
  Ndt(float res, float step_size, float trans_eps, int max_iter);

  bool set_target(const PointCloudPtr & target) override;
  bool match(const PointCloudPtr & input, const Eigen::Matrix4d & initial_pose) override;
  Eigen::Matrix4d get_final_pose() override;
  double get_fitness_score() override;
  void print_info() override;

private:
  bool set_param(float res, float step_size, float trans_eps, int max_iter);
  std::shared_ptr<NormalDistributionsTransform> ndt_;
};

}  // namespace slt_common
