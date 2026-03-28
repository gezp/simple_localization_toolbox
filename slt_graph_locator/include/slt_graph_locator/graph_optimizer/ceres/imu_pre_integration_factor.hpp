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

class ImuPreIntegrationFactor : public ceres::SizedCostFunction<15, 15, 15>
{
public:
  static const int INDEX_P = 0;
  static const int INDEX_R = 3;
  static const int INDEX_V = 6;
  static const int INDEX_A = 9;
  static const int INDEX_G = 12;

  ImuPreIntegrationFactor(
    const Eigen::VectorXd & m, const Eigen::MatrixXd & I, const Eigen::MatrixXd & J,
    const Eigen::Vector3d & g, const double & dt)
  {
    m_ = m;
    I_ = I;
    Eigen::LLT<Eigen::Matrix<double, 15, 15>> LowerI(I_);
    sqrt_info_ = LowerI.matrixL().transpose();
    J_ = J;
    if (J.maxCoeff() > 1e8 || J.minCoeff() < -1e8) {
      std::cout << "numerical unstable in preintegration" << std::endl;
    }
    g_ = g;
    dt_ = dt;
  }

  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    // vertex i
    Eigen::Map<const Eigen::Vector3d> pos_i(&parameters[0][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_i(&parameters[0][INDEX_R]);
    const Sophus::SO3d ori_i = Sophus::SO3d::exp(log_ori_i);
    Eigen::Map<const Eigen::Vector3d> vel_i(&parameters[0][INDEX_V]);
    Eigen::Map<const Eigen::Vector3d> ba_i(&parameters[0][INDEX_A]);
    Eigen::Map<const Eigen::Vector3d> bg_i(&parameters[0][INDEX_G]);
    // vertex j
    Eigen::Map<const Eigen::Vector3d> pos_j(&parameters[1][INDEX_P]);
    Eigen::Map<const Eigen::Vector3d> log_ori_j(&parameters[1][INDEX_R]);
    const Sophus::SO3d ori_j = Sophus::SO3d::exp(log_ori_j);
    Eigen::Map<const Eigen::Vector3d> vel_j(&parameters[1][INDEX_V]);
    Eigen::Map<const Eigen::Vector3d> ba_j(&parameters[1][INDEX_A]);
    Eigen::Map<const Eigen::Vector3d> bg_j(&parameters[1][INDEX_G]);
    // measurement
    const Eigen::Vector3d & alpha_ij = m_.block<3, 1>(INDEX_P, 0);
    const Eigen::Vector3d & theta_ij = m_.block<3, 1>(INDEX_R, 0);
    const Eigen::Vector3d & beta_ij = m_.block<3, 1>(INDEX_V, 0);
    // correct bias
    Eigen::Vector3d dba = ba_i - m_.block<3, 1>(INDEX_A, 0);
    Eigen::Vector3d dbg = bg_i - m_.block<3, 1>(INDEX_G, 0);
    assert(dba.allFinite() && dbg.allFinite());
    Eigen::Matrix3d J_r_bg = J_.block<3, 3>(INDEX_R, INDEX_G);
    Eigen::Matrix3d J_v_ba = J_.block<3, 3>(INDEX_V, INDEX_A);
    Eigen::Matrix3d J_v_bg = J_.block<3, 3>(INDEX_V, INDEX_G);
    Eigen::Matrix3d J_p_ba = J_.block<3, 3>(INDEX_P, INDEX_A);
    Eigen::Matrix3d J_p_bg = J_.block<3, 3>(INDEX_P, INDEX_G);
    Eigen::Vector3d corrected_alpha_ij = alpha_ij + J_p_ba * dba + J_p_bg * dbg;
    Eigen::Vector3d corrected_beta_ij = beta_ij + J_v_ba * dba + J_v_bg * dbg;
    Sophus::SO3d corrected_ori_ij = Sophus::SO3d::exp(theta_ij) * Sophus::SO3d::exp(J_r_bg * dbg);
    // compute residual
    Eigen::Map<Eigen::Vector3d> r_p(residuals + INDEX_P);
    Eigen::Map<Eigen::Vector3d> r_r(residuals + INDEX_R);
    Eigen::Map<Eigen::Vector3d> r_v(residuals + INDEX_V);
    Eigen::Map<Eigen::Vector3d> r_ba(residuals + INDEX_A);
    Eigen::Map<Eigen::Vector3d> r_bg(residuals + INDEX_G);
    r_p =
      ori_i.inverse() * (pos_j - pos_i - vel_i * dt_ - 0.5 * g_ * dt_ * dt_) - corrected_alpha_ij;
    r_r = (corrected_ori_ij.inverse() * ori_i.inverse() * ori_j).log();
    r_v = ori_i.inverse() * (vel_j - vel_i - g_ * dt_) - corrected_beta_ij;
    r_ba = ba_j - ba_i;
    r_bg = bg_j - bg_i;
    Eigen::Map<Eigen::Matrix<double, 15, 1>> r(residuals);
    r = sqrt_info_ * r;
    // compute jacobians
    if (jacobians) {
      Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
      Eigen::Matrix3d Jr_inv = get_jacobian_r_inv(r_r);
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> J_i(jacobians[0]);
        J_i = Eigen::Matrix<double, 15, 15>::Zero();
        // a. residual, position:
        J_i.block<3, 3>(INDEX_P, INDEX_P) = -R_i_inv;
        J_i.block<3, 3>(INDEX_P, INDEX_R) =
          Sophus::SO3d::hat(R_i_inv * (pos_j - pos_i - vel_i * dt_ - 0.5 * g_ * dt_ * dt_));
        J_i.block<3, 3>(INDEX_P, INDEX_V) = -R_i_inv * dt_;
        J_i.block<3, 3>(INDEX_P, INDEX_A) = -J_p_ba;
        J_i.block<3, 3>(INDEX_P, INDEX_G) = -J_p_bg;
        // b. residual, orientation:
        J_i.block<3, 3>(INDEX_R, INDEX_R) = -Jr_inv * (ori_j.inverse() * ori_i).matrix();
        J_i.block<3, 3>(INDEX_R, INDEX_G) = -Jr_inv * Sophus::SO3d::exp(r_r).inverse().matrix() *
          get_jacobian_r(J_r_bg * dbg) * J_r_bg;
        // c. residual, velocity:
        J_i.block<3, 3>(INDEX_V, INDEX_R) = Sophus::SO3d::hat(R_i_inv * (vel_j - vel_i - g_ * dt_));
        J_i.block<3, 3>(INDEX_V, INDEX_V) = -R_i_inv;
        J_i.block<3, 3>(INDEX_V, INDEX_A) = -J_v_ba;
        J_i.block<3, 3>(INDEX_V, INDEX_G) = -J_v_bg;
        // d. residual, bias accel:
        J_i.block<3, 3>(INDEX_A, INDEX_A) = -Eigen::Matrix3d::Identity();
        // e. residual, bias gyro:
        J_i.block<3, 3>(INDEX_G, INDEX_G) = -Eigen::Matrix3d::Identity();
        J_i = sqrt_info_ * J_i;
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> J_j(jacobians[1]);
        J_j = Eigen::Matrix<double, 15, 15>::Zero();
        // a. residual, position:
        J_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;
        // b. residual, orientation:
        J_j.block<3, 3>(INDEX_R, INDEX_R) = Jr_inv;
        // c. residual, velocity:
        J_j.block<3, 3>(INDEX_V, INDEX_V) = R_i_inv;
        // d. residual, bias accel:
        J_j.block<3, 3>(INDEX_A, INDEX_A) = Eigen::Matrix3d::Identity();
        // e. residual, bias gyro:
        J_j.block<3, 3>(INDEX_G, INDEX_G) = Eigen::Matrix3d::Identity();
        J_j = sqrt_info_ * J_j;
      }
    }

    return true;
  }

private:
  Eigen::VectorXd m_;
  Eigen::MatrixXd I_;
  Eigen::Matrix<double, 15, 15> sqrt_info_;
  Eigen::MatrixXd J_;
  Eigen::Vector3d g_ = Eigen::Vector3d::Zero();
  double dt_ = 0.0;
};

}  // namespace slt_graph_locator
