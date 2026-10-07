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

namespace slt_vio
{
// the reprojection error of one sfm observation: camera pose is world to camera, point is in the
// camera frame; rotation is eigen's quaternion order, scalar last, as EigenQuaternionManifold reads
class ReprojectionFactor
{
public:
  ReprojectionFactor(double observed_u, double observed_v)
  : observed_u(observed_u), observed_v(observed_v) {}

  template<typename T>
  bool operator()(
    const T * const camera_q, const T * const camera_t, const T * point, T * residuals) const
  {
    const Eigen::Map<const Eigen::Quaternion<T>> rotation(camera_q);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(camera_t);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> point_camera(point);
    const Eigen::Matrix<T, 3, 1> p = rotation * point_camera + translation;
    residuals[0] = p[0] / p[2] - T(observed_u);
    residuals[1] = p[1] / p[2] - T(observed_v);
    return true;
  }

  static ceres::CostFunction * create(double observed_u, double observed_v)
  {
    return new ceres::AutoDiffCostFunction<ReprojectionFactor, 2, 4, 3, 3>(
      new ReprojectionFactor(observed_u, observed_v));
  }

private:
  double observed_u;
  double observed_v;
};
}  // namespace slt_vio
