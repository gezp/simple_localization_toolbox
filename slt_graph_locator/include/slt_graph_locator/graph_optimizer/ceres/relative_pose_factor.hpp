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

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

#include "slt_graph_locator/graph_optimizer/ceres/utils.hpp"

namespace slt_graph_locator
{

class RelativePoseFactor : public ceres::SizedCostFunction<6, 15, 15>
{
public:
  static const int INDEX_P = 0;
  static const int INDEX_R = 3;

  RelativePoseFactor(const Eigen::VectorXd & m, const Eigen::MatrixXd & I)
  {
    m_ = m;
    I_ = I;
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> LowerI(I_);
    sqrt_info_ = LowerI.matrixL().transpose();
  }

  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    // vertex i
    Eigen::Map<const Eigen::Vector3d> pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d ori_i = Sophus::SO3d::exp(log_ori_i);
    // vertex j
    Eigen::Map<const Eigen::Vector3d> pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d ori_j = Sophus::SO3d::exp(log_ori_j);
    // measurement
    const Eigen::Vector3d & pos_ij = m_.block<3, 1>(INDEX_P, 0);
    const Eigen::Vector3d & log_ori_ij = m_.block<3, 1>(INDEX_R, 0);
    const Sophus::SO3d ori_ij = Sophus::SO3d::exp(log_ori_ij);

    // compute residual
    Eigen::Map<Eigen::Vector3d> r_p(residuals + INDEX_P);
    Eigen::Map<Eigen::Vector3d> r_r(residuals + INDEX_R);
    r_p = ori_i.inverse() * (pos_j - pos_i) - pos_ij;
    r_r = (ori_i.inverse() * ori_j * ori_ij.inverse()).log();
    Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
    r = sqrt_info_ * r;
    // compute jacobians
    if (jacobians) {
      Eigen::Matrix3d R_i = ori_i.matrix();
      Eigen::Matrix3d R_j = ori_j.matrix();
      Eigen::Matrix3d Jr_inv = get_jacobian_r_inv(r_r);
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> J_i(jacobians[0]);
        J_i = Eigen::Matrix<double, 6, 15>::Zero();
        J_i.block<3, 3>(INDEX_P, INDEX_P) = -R_i.transpose();
        J_i.block<3, 3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(R_i.transpose() * (pos_j - pos_i));
        J_i.block<3, 3>(INDEX_R, INDEX_R) = -Jr_inv * ori_ij.matrix() * R_j.transpose() * R_i;
        J_i = sqrt_info_ * J_i;
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> J_j(jacobians[1]);
        J_j = Eigen::Matrix<double, 6, 15>::Zero();
        J_j.block<3, 3>(INDEX_P, INDEX_P) = R_i.transpose();
        J_j.block<3, 3>(INDEX_R, INDEX_R) = Jr_inv * ori_ij.matrix();
        J_j = sqrt_info_ * J_j;
      }
    }
    return true;
  }

private:
  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

}  // namespace slt_graph_locator
