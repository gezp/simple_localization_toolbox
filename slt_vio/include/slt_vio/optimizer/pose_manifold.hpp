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

#include "sophus/so3.hpp"

namespace slt_vio
{
// the pose block of the extrinsic: [p, r], a frame state's retraction without the states beyond its
// pose; the extrinsic is never estimated, so this only satisfies ceres
class PoseManifold : public ceres::Manifold
{
  // the slots of the pose block: the position, then the so3 log of the orientation
  static constexpr int INDEX_P = 0;
  static constexpr int INDEX_R = 3;

public:
  bool Plus(const double * x, const double * delta, double * x_plus_delta) const override
  {
    const Eigen::Map<const Eigen::Vector3d> x_p(x + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> x_r(x + INDEX_R);

    const Eigen::Map<const Eigen::Vector3d> d_p(delta + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> d_r(delta + INDEX_R);

    Eigen::Map<Eigen::Vector3d> out_p(x_plus_delta + INDEX_P);
    Eigen::Map<Eigen::Vector3d> out_r(x_plus_delta + INDEX_R);

    out_p = x_p + d_p;
    out_r = (Sophus::SO3d::exp(x_r) * Sophus::SO3d::exp(d_r)).log();
    return true;
  }

  // the factors write their rotation jacobians against the perturbation itself, so the ambient
  // change is the identity; the right inverse jacobian here would be applied twice (ceres composes)
  bool PlusJacobian(const double *, double * jacobian) const override
  {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
  }

  bool Minus(const double * y, const double * x, double * y_minus_x) const override
  {
    const Eigen::Map<const Eigen::Vector3d> x_p(x + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> x_r(x + INDEX_R);

    const Eigen::Map<const Eigen::Vector3d> y_p(y + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> y_r(y + INDEX_R);

    Eigen::Map<Eigen::Vector3d> d_p(y_minus_x + INDEX_P);
    Eigen::Map<Eigen::Vector3d> d_r(y_minus_x + INDEX_R);

    d_p = y_p - x_p;
    // the right difference, the inverse of Plus: the rotation of x is undone on its own side
    d_r = (Sophus::SO3d::exp(x_r).inverse() * Sophus::SO3d::exp(y_r)).log();
    return true;
  }

  bool MinusJacobian(const double *, double * jacobian) const override
  {
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
  }
  int AmbientSize() const override {return 6;}
  int TangentSize() const override {return 6;}
};
}  // namespace slt_vio
