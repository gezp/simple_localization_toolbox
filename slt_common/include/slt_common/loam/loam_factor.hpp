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
#include <Eigen/Dense>
#include <sophus/so3.hpp>

namespace slt_common
{

class SO3Manifold : public ceres::Manifold
{
public:
  virtual bool Plus(const double * x, const double * delta, double * x_plus_delta) const
  {
    Eigen::Map<const Eigen::Quaterniond> q(x);
    Eigen::Map<const Eigen::Vector3d> delta_so3(delta);
    Eigen::Map<Eigen::Quaterniond> q_plus_delta(x_plus_delta);
    Eigen::Quaterniond delta_q = Sophus::SO3d::exp(delta_so3).unit_quaternion();
    q_plus_delta = (delta_q * q).normalized();
    return true;
  }
  virtual bool PlusJacobian(const double * /*x*/, double * jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    (j.topRows(3)).setIdentity();
    (j.bottomRows(1)).setZero();
    return true;
  }
  virtual int AmbientSize() const {return 4;}
  virtual int TangentSize() const {return 3;}

  virtual bool Minus(const double * y, const double * x, double * y_minus_x) const
  {
    Eigen::Map<const Eigen::Quaterniond> q_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_x(x);
    Eigen::Map<Eigen::Vector3d> delta(y_minus_x);
    delta = (Sophus::SO3d(q_y) * Sophus::SO3d(q_x).inverse()).log();
    return true;
  }

  virtual bool MinusJacobian(const double * x, double * jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
    return true;
  }
};

struct EdgeFactor
{
  EdgeFactor(Eigen::Vector3d current_p_i, Eigen::Vector3d last_p_j, Eigen::Vector3d last_p_l)
  : current_p_i_(current_p_i), last_p_j_(last_p_j), last_p_l_(last_p_l)
  {
  }

  template<typename T>
  bool operator()(const T * q, const T * t, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q_last_current(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_last_current(t);

    Eigen::Matrix<T, 3, 1> p = q_last_current.toRotationMatrix() * current_p_i_ + t_last_current;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> d(residual);
    d = (p - last_p_j_).cross(p - last_p_l_) / (last_p_j_ - last_p_l_).norm();
    return true;
  }

  static ceres::CostFunction * create(
    const Eigen::Vector3d & current_p_i, const Eigen::Vector3d & last_p_j,
    const Eigen::Vector3d & last_p_l)
  {
    return new ceres::AutoDiffCostFunction<EdgeFactor, 3, 4, 3>(
      new EdgeFactor(current_p_i, last_p_j, last_p_l));
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d last_p_j_;
  Eigen::Vector3d last_p_l_;
};

struct PlanarFactor
{
  PlanarFactor(
    Eigen::Vector3d current_p_i, Eigen::Vector3d last_p_j, Eigen::Vector3d last_p_l,
    Eigen::Vector3d last_p_m)
  : current_p_i_(current_p_i), last_p_j_(last_p_j), last_p_l_(last_p_l), last_p_m_(last_p_m)
  {
    last_n_jlm_ = (last_p_j_ - last_p_l_).cross(last_p_j_ - last_p_m_);
    last_n_jlm_.normalize();
  }

  template<typename T>
  bool operator()(const T * q, const T * t, T * residual) const
  {
    Eigen::Map<const Eigen::Quaternion<T>> q_last_current(q);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_last_current(t);
    Eigen::Matrix<T, 3, 1> p = q_last_current.toRotationMatrix() * current_p_i_ + t_last_current;

    residual[0] = (p - last_p_j_).dot(last_n_jlm_);
    return true;
  }

  static ceres::CostFunction * create(
    const Eigen::Vector3d current_p_i, const Eigen::Vector3d last_p_j,
    const Eigen::Vector3d last_p_l, const Eigen::Vector3d last_p_m)
  {
    return new ceres::AutoDiffCostFunction<PlanarFactor, 1, 4, 3>(
      new PlanarFactor(current_p_i, last_p_j, last_p_l, last_p_m));
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d last_p_j_;
  Eigen::Vector3d last_p_l_;
  Eigen::Vector3d last_p_m_;
  Eigen::Vector3d last_n_jlm_;
};

class EdgeAnalyticFactor : public ceres::SizedCostFunction<3, 4, 3>
{
public:
  EdgeAnalyticFactor(
    Eigen::Vector3d current_p_i, Eigen::Vector3d last_p_j, Eigen::Vector3d last_p_l)
  : current_p_i_(current_p_i), last_p_j_(last_p_j), last_p_l_(last_p_l)
  {
  }
  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    Eigen::Map<const Eigen::Quaterniond> q_last_current(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_current(parameters[1]);

    Eigen::Vector3d p = q_last_current * current_p_i_ + t_last_current;

    Eigen::Map<Eigen::Vector3d> d(residuals);
    d = (p - last_p_j_).cross(p - last_p_l_) / (last_p_j_ - last_p_l_).norm();

    if (jacobians) {
      Eigen::Vector3d v = last_p_l_ - last_p_j_;
      Eigen::Matrix3d de_by_dp = (Sophus::SO3d::hat(v) / v.norm());
      if (jacobians[0]) {
        // R
        Eigen::Matrix3d dp_by_so3 = -Sophus::SO3d::hat(q_last_current * current_p_i_);
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> J_q(jacobians[0]);
        J_q.setZero();
        J_q.block<3, 3>(0, 0) = de_by_dp * dp_by_so3;
      }
      if (jacobians[1]) {
        // t
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_t(jacobians[1]);
        J_t.setZero();
        J_t = de_by_dp * Eigen::Matrix3d::Identity();
      }
    }
    return true;
  }
  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d last_p_j_;
  Eigen::Vector3d last_p_l_;
};

class PlanarAnalyticFactor : public ceres::SizedCostFunction<1, 4, 3>
{
public:
  PlanarAnalyticFactor(
    Eigen::Vector3d current_p_i, Eigen::Vector3d last_p_j, Eigen::Vector3d last_p_l,
    Eigen::Vector3d last_p_m)
  : current_p_i_(current_p_i), last_p_j_(last_p_j), last_p_l_(last_p_l), last_p_m_(last_p_m)
  {
    last_n_jlm_ = (last_p_j_ - last_p_l_).cross(last_p_j_ - last_p_m_);
    last_n_jlm_.normalize();
  }
  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    Eigen::Map<const Eigen::Quaterniond> q_last_current(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> t_last_current(parameters[1]);

    Eigen::Vector3d p = q_last_current * current_p_i_ + t_last_current;

    residuals[0] = (p - last_p_j_).dot(last_n_jlm_);

    if (jacobians) {
      Eigen::Matrix<double, 1, 3> dh_by_dp = last_n_jlm_.transpose();
      if (jacobians[0]) {
        // R
        Eigen::Matrix3d dp_by_so3 = -Sophus::SO3d::hat(q_last_current * current_p_i_);
        Eigen::Map<Eigen::Matrix<double, 1, 4>> J_q(jacobians[0]);
        J_q.setZero();
        J_q.block<1, 3>(0, 0) = dh_by_dp * dp_by_so3;
      }
      if (jacobians[1]) {
        // t
        Eigen::Map<Eigen::Matrix<double, 1, 3>> J_t(jacobians[1]);
        J_t = dh_by_dp * Eigen::Matrix3d::Identity();
      }
    }
    return true;
  }

  Eigen::Vector3d current_p_i_;
  Eigen::Vector3d last_p_j_;
  Eigen::Vector3d last_p_l_;
  Eigen::Vector3d last_p_m_;
  Eigen::Vector3d last_n_jlm_;
};

}  // namespace slt_common
