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

namespace slt_vio
{
// the frame state block [p, r, v, ba, bg], the so3 log of the orientation in the middle, perturbed
// [dp, delta, dv, dba, dbg] with the rotation retracted on the right (r * exp(delta)); ambient size
// equals tangent size. the layout the graph locator retracts its states by
class PrvagManifold : public ceres::Manifold
{
  // the state block slots, the order the graph locator lays its states out in
  static constexpr int INDEX_P = 0;
  static constexpr int INDEX_R = 3;
  static constexpr int INDEX_V = 6;
  static constexpr int INDEX_BA = 9;
  static constexpr int INDEX_BG = 12;

public:
  bool Plus(const double * x, const double * delta, double * x_plus_delta) const override
  {
    const Eigen::Map<const Eigen::Vector3d> x_p(x + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> x_r(x + INDEX_R);
    const Eigen::Map<const Eigen::Vector3d> x_v(x + INDEX_V);
    const Eigen::Map<const Eigen::Vector3d> x_ba(x + INDEX_BA);
    const Eigen::Map<const Eigen::Vector3d> x_bg(x + INDEX_BG);

    const Eigen::Map<const Eigen::Vector3d> d_p(delta + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> d_r(delta + INDEX_R);
    const Eigen::Map<const Eigen::Vector3d> d_v(delta + INDEX_V);
    const Eigen::Map<const Eigen::Vector3d> d_ba(delta + INDEX_BA);
    const Eigen::Map<const Eigen::Vector3d> d_bg(delta + INDEX_BG);

    Eigen::Map<Eigen::Vector3d> out_p(x_plus_delta + INDEX_P);
    Eigen::Map<Eigen::Vector3d> out_r(x_plus_delta + INDEX_R);
    Eigen::Map<Eigen::Vector3d> out_v(x_plus_delta + INDEX_V);
    Eigen::Map<Eigen::Vector3d> out_ba(x_plus_delta + INDEX_BA);
    Eigen::Map<Eigen::Vector3d> out_bg(x_plus_delta + INDEX_BG);

    // only the rotation is not plain addition
    out_p = x_p + d_p;
    out_r = (Sophus::SO3d::exp(x_r) * Sophus::SO3d::exp(d_r)).log();
    out_v = x_v + d_v;
    out_ba = x_ba + d_ba;
    out_bg = x_bg + d_bg;
    return true;
  }

  // the factors write their rotation jacobians against the perturbation itself, so the ambient
  // change is the identity; the right inverse jacobian here would be applied twice (ceres composes)
  bool PlusJacobian(const double *, double * jacobian) const override
  {
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
  }

  bool Minus(const double * y, const double * x, double * y_minus_x) const override
  {
    const Eigen::Map<const Eigen::Vector3d> x_p(x + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> x_r(x + INDEX_R);
    const Eigen::Map<const Eigen::Vector3d> x_v(x + INDEX_V);
    const Eigen::Map<const Eigen::Vector3d> x_ba(x + INDEX_BA);
    const Eigen::Map<const Eigen::Vector3d> x_bg(x + INDEX_BG);

    const Eigen::Map<const Eigen::Vector3d> y_p(y + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> y_r(y + INDEX_R);
    const Eigen::Map<const Eigen::Vector3d> y_v(y + INDEX_V);
    const Eigen::Map<const Eigen::Vector3d> y_ba(y + INDEX_BA);
    const Eigen::Map<const Eigen::Vector3d> y_bg(y + INDEX_BG);

    Eigen::Map<Eigen::Vector3d> d_p(y_minus_x + INDEX_P);
    Eigen::Map<Eigen::Vector3d> d_r(y_minus_x + INDEX_R);
    Eigen::Map<Eigen::Vector3d> d_v(y_minus_x + INDEX_V);
    Eigen::Map<Eigen::Vector3d> d_ba(y_minus_x + INDEX_BA);
    Eigen::Map<Eigen::Vector3d> d_bg(y_minus_x + INDEX_BG);

    d_p = y_p - x_p;
    // the right difference, the inverse of Plus: the rotation of x is undone on its own side
    d_r = (Sophus::SO3d::exp(x_r).inverse() * Sophus::SO3d::exp(y_r)).log();
    d_v = y_v - x_v;
    d_ba = y_ba - x_ba;
    d_bg = y_bg - x_bg;
    return true;
  }

  bool MinusJacobian(const double *, double * jacobian) const override
  {
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
  }

  int AmbientSize() const override {return 15;}
  int TangentSize() const override {return 15;}
};
}  // namespace slt_vio
