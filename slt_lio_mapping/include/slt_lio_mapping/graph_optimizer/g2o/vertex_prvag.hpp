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

/*
 * @Description: g2o vertex for LIO extended pose
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#pragma once

#include <g2o/core/base_vertex.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <mutex>
#include <sophus/so3.hpp>

namespace g2o
{

struct PRVAG
{
  static const int INDEX_POS = 0;
  static const int INDEX_ORI = 3;
  static const int INDEX_VEL = 6;
  static const int INDEX_B_A = 9;
  static const int INDEX_B_G = 12;

  PRVAG() {}

  explicit PRVAG(const double * data)
  {
    pos = Eigen::Vector3d(data[INDEX_POS + 0], data[INDEX_POS + 1], data[INDEX_POS + 2]);
    ori = Sophus::SO3d::exp(
      Eigen::Vector3d(data[INDEX_ORI + 0], data[INDEX_ORI + 1], data[INDEX_ORI + 2]));
    vel = Eigen::Vector3d(data[INDEX_VEL + 0], data[INDEX_VEL + 1], data[INDEX_VEL + 2]);
    b_a = Eigen::Vector3d(data[INDEX_B_A + 0], data[INDEX_B_A + 1], data[INDEX_B_A + 2]);
    b_g = Eigen::Vector3d(data[INDEX_B_G + 0], data[INDEX_B_G + 1], data[INDEX_B_G + 2]);
  }

  void write(double * data) const
  {
    // get orientation in so3:
    auto log_ori = ori.log();

    for (size_t i = 0; i < 3; ++i) {
      data[INDEX_POS + i] = pos(i);
      data[INDEX_ORI + i] = log_ori(i);
      data[INDEX_VEL + i] = vel(i);
      data[INDEX_B_A + i] = b_a(i);
      data[INDEX_B_G + i] = b_g(i);
    }
  }

  double time = 0.0;

  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Sophus::SO3d ori = Sophus::SO3d();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
};

class VertexPRVAG : public g2o::BaseVertex<15, PRVAG>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override {_estimate = PRVAG();}

  void oplusImpl(const double * update) override
  {
    //
    Eigen::Vector3d delta_pos(
      update[PRVAG::INDEX_POS + 0], update[PRVAG::INDEX_POS + 1], update[PRVAG::INDEX_POS + 2]);
    Eigen::Vector3d delta_ori(
      update[PRVAG::INDEX_ORI + 0], update[PRVAG::INDEX_ORI + 1], update[PRVAG::INDEX_ORI + 2]);
    Eigen::Vector3d delta_vel(
      update[PRVAG::INDEX_VEL + 0], update[PRVAG::INDEX_VEL + 1], update[PRVAG::INDEX_VEL + 2]);
    Eigen::Vector3d delta_b_a(
      update[PRVAG::INDEX_B_A + 0], update[PRVAG::INDEX_B_A + 1], update[PRVAG::INDEX_B_A + 2]);
    Eigen::Vector3d delta_b_g(
      update[PRVAG::INDEX_B_G + 0], update[PRVAG::INDEX_B_G + 1], update[PRVAG::INDEX_B_G + 2]);
    _estimate.pos += delta_pos;
    _estimate.ori = _estimate.ori * Sophus::SO3d::exp(delta_ori);
    _estimate.vel += delta_vel;
    _estimate.b_a += delta_b_a;
    _estimate.b_g += delta_b_g;
    updateDeltaBiases(delta_b_a, delta_b_g);
  }

  bool isUpdated(void) const {return _is_updated;}

  void updateDeltaBiases(const Eigen::Vector3d & d_b_a_i, const Eigen::Vector3d & d_b_g_i)
  {
    std::lock_guard<std::mutex> l(_m);

    _is_updated = true;

    _d_b_a_i += d_b_a_i;
    _d_b_g_i += d_b_g_i;
  }

  void getDeltaBiases(Eigen::Vector3d & d_b_a_i, Eigen::Vector3d & d_b_g_i)
  {
    std::lock_guard<std::mutex> l(_m);

    d_b_a_i = _d_b_a_i;
    d_b_g_i = _d_b_g_i;

    _d_b_a_i = _d_b_g_i = Eigen::Vector3d::Zero();

    _is_updated = false;
  }

  virtual bool read(std::istream & in) {return true;}

  virtual bool write(std::ostream & out) const {return true;}

private:
  std::mutex _m;
  bool _is_updated = false;

  Eigen::Vector3d _d_b_a_i = Eigen::Vector3d::Zero();
  Eigen::Vector3d _d_b_g_i = Eigen::Vector3d::Zero();
};

}  // namespace g2o
