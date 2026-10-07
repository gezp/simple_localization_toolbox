// Copyright 2026 Zhenpeng Ge (https://github.com/gezp).
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

#include <Eigen/Dense>
#include <cmath>

#include <sophus/so3.hpp>

namespace slt_common
{
// rotation helpers shared by the estimator, its factors and the initializer

// so3 right jacobian and its inverse; the perturbation tangent is right (R * exp(delta)), so
// every rotation jacobian goes through these. both fall back to identity for small theta
inline Eigen::Matrix3d get_jacobian_r(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d j_r = Eigen::Matrix3d::Identity();
  const double theta = w.norm();
  if (theta > 1e-5) {
    const Eigen::Vector3d a = w.normalized();
    const Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
    j_r = std::sin(theta) / theta * Eigen::Matrix3d::Identity() +
      (1.0 - std::sin(theta) / theta) * a * a.transpose() -
      (1.0 - std::cos(theta)) / theta * a_hat;
  }
  return j_r;
}

inline Eigen::Matrix3d get_jacobian_r_inv(const Eigen::Vector3d & w)
{
  Eigen::Matrix3d j_r_inv = Eigen::Matrix3d::Identity();
  const double theta = w.norm();
  if (theta > 1e-5) {
    const Eigen::Vector3d a = w.normalized();
    const Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
    const double theta_half = 0.5 * theta;
    const double cot_theta = 1.0 / std::tan(theta_half);
    j_r_inv = theta_half * cot_theta * Eigen::Matrix3d::Identity() +
      (1.0 - theta_half * cot_theta) * a * a.transpose() + theta_half * a_hat;
  }
  return j_r_inv;
}

// roll pitch yaw of the rotation, in degree
inline Eigen::Vector3d R_to_ypr(const Eigen::Matrix3d & r)
{
  const Eigen::Vector3d n = r.col(0);
  const Eigen::Vector3d o = r.col(1);
  const Eigen::Vector3d a = r.col(2);
  const double y = std::atan2(n(1), n(0));
  const double p = std::atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
  const double r_ = std::atan2(a(0) * std::sin(y) - a(1) * std::cos(y),
      -o(0) * std::sin(y) + o(1) * std::cos(y));
  return Eigen::Vector3d{y, p, r_} / M_PI * 180.0;
}

// inverse of R_to_ypr, the angles are in degree
inline Eigen::Matrix3d ypr_to_R(const Eigen::Vector3d & ypr)
{
  const double y = ypr(0) / 180.0 * M_PI;
  const double p = ypr(1) / 180.0 * M_PI;
  const double r = ypr(2) / 180.0 * M_PI;
  Eigen::Matrix3d rz;
  rz << std::cos(y), -std::sin(y), 0,
    std::sin(y), std::cos(y), 0,
    0, 0, 1;
  Eigen::Matrix3d ry;
  ry << std::cos(p), 0, std::sin(p),
    0, 1, 0,
    -std::sin(p), 0, std::cos(p);
  Eigen::Matrix3d rx;
  rx << 1, 0, 0,
    0, std::cos(r), -std::sin(r),
    0, std::sin(r), std::cos(r);
  return rz * ry * rx;
}

}  // namespace slt_common
