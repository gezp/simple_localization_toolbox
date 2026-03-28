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
class EdgeSE3PriorQuat : public g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PriorQuat()
  : g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3>() {}

  void computeError() override
  {
    const g2o::VertexSE3 * v1 = static_cast<const g2o::VertexSE3 *>(_vertices[0]);

    Eigen::Quaterniond estimate = Eigen::Quaterniond(v1->estimate().linear());
    if (estimate.w() < 0) {
      estimate.coeffs() = -estimate.coeffs();
    }
    _error = estimate.vec() - _measurement.vec();
  }

  void setMeasurement(const Eigen::Quaterniond & m) override
  {
    _measurement = m;
    if (m.w() < 0.0) {
      _measurement.coeffs() = -m.coeffs();
    }
  }

  bool read(std::istream & is) override
  {
    Eigen::Quaterniond q;
    is >> q.w() >> q.x() >> q.y() >> q.z();
    setMeasurement(q);
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if (i != j) {
          information()(j, i) = information()(i, j);
        }
      }
    }
    return true;
  }

  bool write(std::ostream & os) const override
  {
    Eigen::Quaterniond q = _measurement;
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
    for (int i = 0; i < information().rows(); ++i) {
      for (int j = i; j < information().cols(); ++j) {
        os << " " << information()(i, j);
      }
    }
    return os.good();
  }
};
}  // namespace g2o
