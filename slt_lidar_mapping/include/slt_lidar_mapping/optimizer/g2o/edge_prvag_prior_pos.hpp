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
 * @Description: g2o edge for LIO GNSS measurement
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */
#pragma once

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include "slt_lidar_mapping/optimizer/g2o/vertex_prvag.hpp"

namespace g2o
{

class EdgePRVAGPriorPos : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPRVAG>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgePRVAGPriorPos()
  : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexPRVAG>() {}

  void computeError() override
  {
    const g2o::VertexPRVAG * v = static_cast<const g2o::VertexPRVAG *>(_vertices[0]);

    const Eigen::Vector3d & estimate = v->estimate().pos;

    _error = estimate - _measurement;
  }

  void setMeasurement(const Eigen::Vector3d & m) override {_measurement = m;}

  bool read(std::istream & is) override
  {
    Eigen::Vector3d v;

    is >> v(0) >> v(1) >> v(2);

    setMeasurement(v);

    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        // update cross-diagonal element:
        if (i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }

    return true;
  }

  bool write(std::ostream & os) const override
  {
    Eigen::Vector3d v = _measurement;

    os << v(0) << " " << v(1) << " " << v(2) << " ";

    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      }
    }

    return os.good();
  }
};

}  // namespace g2o
