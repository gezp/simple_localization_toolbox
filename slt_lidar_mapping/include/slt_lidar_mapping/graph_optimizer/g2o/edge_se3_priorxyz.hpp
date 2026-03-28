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

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o
{
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PriorXYZ()
  : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {}

  void computeError() override
  {
    const g2o::VertexSE3 * v1 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);

    Eigen::Vector3d estimate = v1->estimate().translation();
    _error = estimate - _measurement;
  }

  void setMeasurement(const Eigen::Vector3d & m) override {_measurement = m;}

  bool read(std::istream & is) override
  {
    Eigen::Vector3d v;
    is >> v(0) >> v(1) >> v(2);

    setMeasurement(Eigen::Vector3d(v));

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
