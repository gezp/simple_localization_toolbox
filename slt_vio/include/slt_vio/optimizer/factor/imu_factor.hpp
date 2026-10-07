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

#include "ceres/ceres.h"
#include "slt_common/math_utils.hpp"
#include "slt_imu_odometry/imu_pre_integration.hpp"

namespace slt_vio
{
// the 15 dim preintegration residual between two consecutive window states. the covariance whitens
// it and the jacobians are hand written against the state block's so3 right perturbation.
// gravity is the world one refined at init; the preintegration's acceleration is specific force
class ImuFactor : public ceres::SizedCostFunction<15, 15, 15>
{
  // the state block slots, the same offsets the 15 dim residual is laid out in
  static constexpr int INDEX_P = 0;
  static constexpr int INDEX_R = 3;
  static constexpr int INDEX_V = 6;
  static constexpr int INDEX_BA = 9;
  static constexpr int INDEX_BG = 12;

public:
  ImuFactor(
    const slt_imu_odometry::ImuPreIntegrationPtr & pre_integration, const Eigen::Vector3d & gravity)
  : pre_integration_(pre_integration), gravity_(gravity) {}

  bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const override;

private:
  slt_imu_odometry::ImuPreIntegrationPtr pre_integration_;
  Eigen::Vector3d gravity_;
};

inline bool ImuFactor::Evaluate(
  double const * const * parameters, double * residuals, double ** jacobians) const
{
  const Eigen::Map<const Eigen::Vector3d> p_i(parameters[0] + INDEX_P);
  const Eigen::Map<const Eigen::Vector3d> r_i(parameters[0] + INDEX_R);
  const Eigen::Map<const Eigen::Vector3d> v_i(parameters[0] + INDEX_V);
  const Eigen::Map<const Eigen::Vector3d> ba_i(parameters[0] + INDEX_BA);
  const Eigen::Map<const Eigen::Vector3d> bg_i(parameters[0] + INDEX_BG);
  const Sophus::SO3d R_i = Sophus::SO3d::exp(r_i);

  const Eigen::Map<const Eigen::Vector3d> p_j(parameters[1] + INDEX_P);
  const Eigen::Map<const Eigen::Vector3d> r_j(parameters[1] + INDEX_R);
  const Eigen::Map<const Eigen::Vector3d> v_j(parameters[1] + INDEX_V);
  const Eigen::Map<const Eigen::Vector3d> ba_j(parameters[1] + INDEX_BA);
  const Eigen::Map<const Eigen::Vector3d> bg_j(parameters[1] + INDEX_BG);
  const Sophus::SO3d R_j = Sophus::SO3d::exp(r_j);

  // the preintegration is linearized at the biases set_bias was given, the residual is shifted to
  // the iteration's biases with the first order jacobians
  const Eigen::Matrix<double, 15, 15> jacobian = pre_integration_->get_jacobian();
  const Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(INDEX_P, INDEX_BA);
  const Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(INDEX_P, INDEX_BG);
  const Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(INDEX_R, INDEX_BG);
  const Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(INDEX_V, INDEX_BA);
  const Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(INDEX_V, INDEX_BG);

  const Eigen::Vector3d dba = ba_i - pre_integration_->get_ba();
  const Eigen::Vector3d dbg = bg_i - pre_integration_->get_bg();
  // the measurement carries the rotation as a matrix; the bias correction is the so3 exp of the
  // tangent jacobian the preintegration propagated
  const Sophus::SO3d corrected_delta_q =
    Sophus::SO3d(pre_integration_->get_theta()) * Sophus::SO3d::exp(dq_dbg * dbg);
  const Eigen::Vector3d corrected_delta_v =
    pre_integration_->get_beta() + dv_dba * dba + dv_dbg * dbg;
  const Eigen::Vector3d corrected_delta_p =
    pre_integration_->get_alpha() + dp_dba * dba + dp_dbg * dbg;
  const double sum_dt = pre_integration_->get_delta_time();
  const Eigen::Vector3d & gravity = gravity_;

  // the rotation residual, held before whitening: its jacobians below are taken where it stands,
  // and whitening mixes its rows
  const Eigen::Vector3d r_r = (corrected_delta_q.inverse() * R_i.inverse() * R_j).log();

  Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
  residual.block<3, 1>(INDEX_P, 0) =
    R_i.inverse() * (p_j - p_i - v_i * sum_dt - 0.5 * gravity * sum_dt * sum_dt) -
    corrected_delta_p;
  residual.block<3, 1>(INDEX_R, 0) = r_r;
  residual.block<3, 1>(INDEX_V, 0) =
    R_i.inverse() * (v_j - v_i - gravity * sum_dt) - corrected_delta_v;
  residual.block<3, 1>(INDEX_BA, 0) = ba_j - ba_i;
  residual.block<3, 1>(INDEX_BG, 0) = bg_j - bg_i;

  const Eigen::Matrix<double, 15, 15> sqrt_info =
    Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->get_covariance().inverse())
    .matrixL().transpose();
  residual = sqrt_info * residual;

  if (!jacobians) {
    return true;
  }
  const Eigen::Matrix3d R_i_inv = R_i.inverse().matrix();
  const Eigen::Matrix3d jr_inv = slt_common::get_jacobian_r_inv(r_r);
  const Eigen::Vector3d dp_i = p_j - p_i - v_i * sum_dt - 0.5 * gravity * sum_dt * sum_dt;
  const Eigen::Vector3d dv_i = v_j - v_i - gravity * sum_dt;

  // the state block is the residual itself, so a jacobian column is the state offset of the row
  if (jacobians[0]) {
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_i(jacobians[0]);
    jacobian_i.setZero();
    jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -R_i_inv;
    jacobian_i.block<3, 3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(R_i_inv * dp_i);
    jacobian_i.block<3, 3>(INDEX_P, INDEX_V) = -R_i_inv * sum_dt;
    jacobian_i.block<3, 3>(INDEX_P, INDEX_BA) = -dp_dba;
    jacobian_i.block<3, 3>(INDEX_P, INDEX_BG) = -dp_dbg;
    jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -jr_inv * R_j.matrix().transpose() * R_i.matrix();
    jacobian_i.block<3, 3>(INDEX_R, INDEX_BG) =
      -jr_inv * Sophus::SO3d::exp(r_r).inverse().matrix() *
      slt_common::get_jacobian_r(dq_dbg * dbg) * dq_dbg;
    jacobian_i.block<3, 3>(INDEX_V, INDEX_R) = Sophus::SO3d::hat(R_i_inv * dv_i);
    jacobian_i.block<3, 3>(INDEX_V, INDEX_V) = -R_i_inv;
    jacobian_i.block<3, 3>(INDEX_V, INDEX_BA) = -dv_dba;
    jacobian_i.block<3, 3>(INDEX_V, INDEX_BG) = -dv_dbg;
    jacobian_i.block<3, 3>(INDEX_BA, INDEX_BA) = -Eigen::Matrix3d::Identity();
    jacobian_i.block<3, 3>(INDEX_BG, INDEX_BG) = -Eigen::Matrix3d::Identity();
    jacobian_i = sqrt_info * jacobian_i;
  }
  if (jacobians[1]) {
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
    jacobian_j.setZero();
    jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;
    jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = jr_inv;
    jacobian_j.block<3, 3>(INDEX_V, INDEX_V) = R_i_inv;
    jacobian_j.block<3, 3>(INDEX_BA, INDEX_BA) = Eigen::Matrix3d::Identity();
    jacobian_j.block<3, 3>(INDEX_BG, INDEX_BG) = Eigen::Matrix3d::Identity();
    jacobian_j = sqrt_info * jacobian_j;
  }
  return true;
}
}  // namespace slt_vio
