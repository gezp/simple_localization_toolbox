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

#include "ceres/ceres.h"

#include "slt_common/math_utils.hpp"

namespace slt_vio
{
// the reprojection error of one feature seen by two window frames; the points are normalized camera
// coordinates and the depth an inverse depth
class ProjectionFactor
  : public ceres::SizedCostFunction<2, 15, 15, 6, 1>
{
  // the pose slots a state block the residual reads: position, then the so3 log of the rotation
  static constexpr int INDEX_P = 0;
  static constexpr int INDEX_R = 3;

public:
  ProjectionFactor(
    const Eigen::Vector3d & pts_i, const Eigen::Vector3d & pts_j,
    const Eigen::Matrix2d & sqrt_info)
  : pts_i_(pts_i), pts_j_(pts_j), sqrt_info_(sqrt_info) {}

  bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const override;

private:
  Eigen::Vector3d pts_i_;
  Eigen::Vector3d pts_j_;
  Eigen::Matrix2d sqrt_info_;
};

inline bool ProjectionFactor::Evaluate(
  double const * const * parameters, double * residuals, double ** jacobians) const
{
  const Eigen::Map<const Eigen::Vector3d> p_i(parameters[0] + INDEX_P);
  const Eigen::Map<const Eigen::Vector3d> r_i(parameters[0] + INDEX_R);
  const Eigen::Matrix3d Ri = Sophus::SO3d::exp(r_i).matrix();

  const Eigen::Map<const Eigen::Vector3d> p_j(parameters[1] + INDEX_P);
  const Eigen::Map<const Eigen::Vector3d> r_j(parameters[1] + INDEX_R);
  const Eigen::Matrix3d Rj = Sophus::SO3d::exp(r_j).matrix();

  // the extrinsic block: the translation, then the so3 log of the rotation
  const Eigen::Map<const Eigen::Vector3d> t_bc(parameters[2]);
  const Eigen::Map<const Eigen::Vector3d> r_bc(parameters[2] + 3);
  const Eigen::Matrix3d R_bc = Sophus::SO3d::exp(r_bc).matrix();

  const double inv_dep_i = parameters[3][0];
  // a depth of zero, or one that is not a number any more, is refused: the division below would
  // poison the solve with an infinite step
  if (!std::isfinite(inv_dep_i) || inv_dep_i == 0.0) {
    return false;
  }

  // the feature is lifted to the world with the first observation's depth, then reprojected into
  // the second frame
  const Eigen::Vector3d pts_camera_i = pts_i_ / inv_dep_i;
  const Eigen::Vector3d pts_imu_i = R_bc * pts_camera_i + t_bc;
  const Eigen::Vector3d pts_w = Ri * pts_imu_i + p_i;
  const Eigen::Vector3d pts_imu_j = Rj.transpose() * (pts_w - p_j);
  const Eigen::Vector3d pts_camera_j = R_bc.transpose() * (pts_imu_j - t_bc);
  const double dep_j = pts_camera_j.z();
  if (!std::isfinite(dep_j) || dep_j == 0.0) {
    return false;
  }

  Eigen::Map<Eigen::Vector2d> residual(residuals);
  residual = (pts_camera_j / dep_j).head<2>() - pts_j_.head<2>();
  residual = sqrt_info_ * residual;

  if (!jacobians) {
    return true;
  }
  Eigen::Matrix<double, 2, 3> reduce;
  reduce << 1.0 / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
    0, 1.0 / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
  reduce = sqrt_info_ * reduce;

  // a state block is wider than the pose the residual reads; the rest of its columns are the states
  // this residual does not touch
  if (jacobians[0]) {
    Eigen::Map<Eigen::Matrix<double, 2, 15, Eigen::RowMajor>> jacobian_state_i(
      jacobians[0]);
    Eigen::Matrix<double, 3, 6> jaco_i;
    jaco_i.leftCols<3>() = R_bc.transpose() * Rj.transpose();
    jaco_i.rightCols<3>() = R_bc.transpose() * Rj.transpose() * Ri * -Sophus::SO3d::hat(pts_imu_i);
    jacobian_state_i.setZero();
    jacobian_state_i.leftCols<6>() = reduce * jaco_i;
  }
  if (jacobians[1]) {
    Eigen::Map<Eigen::Matrix<double, 2, 15, Eigen::RowMajor>> jacobian_state_j(
      jacobians[1]);
    Eigen::Matrix<double, 3, 6> jaco_j;
    jaco_j.leftCols<3>() = R_bc.transpose() * -Rj.transpose();
    jaco_j.rightCols<3>() = R_bc.transpose() * Sophus::SO3d::hat(pts_imu_j);
    jacobian_state_j.setZero();
    jacobian_state_j.leftCols<6>() = reduce * jaco_j;
  }
  if (jacobians[2]) {
    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jacobian_ex_pose(
      jacobians[2]);
    Eigen::Matrix<double, 3, 6> jaco_ex;
    jaco_ex.leftCols<3>() = R_bc.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
    const Eigen::Matrix3d tmp_R = R_bc.transpose() * Rj.transpose() * Ri * R_bc;
    jaco_ex.rightCols<3>() = -tmp_R * Sophus::SO3d::hat(pts_camera_i) +
      Sophus::SO3d::hat(tmp_R * pts_camera_i) +
      Sophus::SO3d::hat(R_bc.transpose() * (Rj.transpose() * (Ri * t_bc + p_i - p_j) - t_bc));
    jacobian_ex_pose = reduce * jaco_ex;
  }
  if (jacobians[3]) {
    Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
    jacobian_feature =
      reduce * R_bc.transpose() * Rj.transpose() * Ri * R_bc * pts_i_ * -1.0 /
      (inv_dep_i * inv_dep_i);
  }
  return true;
}
}  // namespace slt_vio
