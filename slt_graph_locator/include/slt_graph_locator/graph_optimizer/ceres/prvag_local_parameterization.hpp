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

namespace slt_graph_locator
{

class PrvagLocalParameterization : public ceres::LocalParameterization
{
public:
  static const int INDEX_P = 0;
  static const int INDEX_R = 3;
  static const int INDEX_V = 6;
  static const int INDEX_A = 9;
  static const int INDEX_G = 12;

  virtual bool Plus(const double * x, const double * delta, double * x_plus_delta) const
  {
    Eigen::Map<const Eigen::Vector3d> pos(x + INDEX_P);
    Eigen::Map<const Eigen::Vector3d> ori(x + INDEX_R);
    Eigen::Map<const Eigen::Vector3d> vel(x + INDEX_V);
    Eigen::Map<const Eigen::Vector3d> ba(x + INDEX_A);
    Eigen::Map<const Eigen::Vector3d> bg(x + INDEX_G);

    Eigen::Map<const Eigen::Vector3d> d_pos(delta + INDEX_P);
    Eigen::Map<const Eigen::Vector3d> d_ori(delta + INDEX_R);
    Eigen::Map<const Eigen::Vector3d> d_vel(delta + INDEX_V);
    Eigen::Map<const Eigen::Vector3d> d_ba(delta + INDEX_A);
    Eigen::Map<const Eigen::Vector3d> d_bg(delta + INDEX_G);

    Eigen::Map<Eigen::Vector3d> pos_plus(x_plus_delta + INDEX_P);
    Eigen::Map<Eigen::Vector3d> ori_plus(x_plus_delta + INDEX_R);
    Eigen::Map<Eigen::Vector3d> vel_plus(x_plus_delta + INDEX_V);
    Eigen::Map<Eigen::Vector3d> ba_plus(x_plus_delta + INDEX_A);
    Eigen::Map<Eigen::Vector3d> bg_plus(x_plus_delta + INDEX_G);

    pos_plus = pos + d_pos;

    ori_plus = (Sophus::SO3d::exp(ori) * Sophus::SO3d::exp(d_ori)).log();
    vel_plus = vel + d_vel;
    ba_plus = ba + d_ba;
    bg_plus = bg + d_bg;

    return true;
  }

  virtual bool ComputeJacobian(const double * /*x*/, double * jacobian) const
  {
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> J(jacobian);
    J.setIdentity();
    return true;
  }

  virtual int GlobalSize() const {return 15;}

  virtual int LocalSize() const {return 15;}
};

}  // namespace slt_graph_locator
