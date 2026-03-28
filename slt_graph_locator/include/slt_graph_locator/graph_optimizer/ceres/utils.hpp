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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

namespace slt_graph_locator
{

Eigen::Matrix3d get_jacobian_r_inv(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();
  double theta = w.norm();
  if (theta > 1e-5) {
    Eigen::Vector3d a = w.normalized();
    Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
    double theta_half = 0.5 * theta;
    double cot_theta = 1.0 / tan(theta_half);
    J_r_inv = theta_half * cot_theta * J_r_inv +
      (1.0 - theta_half * cot_theta) * a * a.transpose() + theta_half * a_hat;
  }
  return J_r_inv;
}

Eigen::Matrix3d get_jacobian_r(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d J_r = Eigen::Matrix3d::Identity();
  double theta = w.norm();
  if (theta > 1e-5) {
    Eigen::Vector3d a = w.normalized();
    Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
    J_r = sin(theta) / theta * Eigen::Matrix3d::Identity() +
      (1.0 - sin(theta) / theta) * a * a.transpose() - (1.0 - cos(theta)) / theta * a_hat;
  }
  return J_r;
}

}  // namespace slt_graph_locator
