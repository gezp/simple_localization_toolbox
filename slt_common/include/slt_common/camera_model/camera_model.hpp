// Copyright 2026 Gezp (https://github.com/gezp).
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
#include <vector>

#include <opencv2/core.hpp>

#include "slt_common/camera_model/camera_intrinsic_param.hpp"

namespace slt_common
{
// pinhole camera with the radial-tangential (plumb_bob) distortion model; the undistortion
// is iterative and delegated to cv::undistortPoints
class CameraModel
{
public:
  // false on an unusable parameter (unknown model or non-positive focal length); the model
  // stays invalid and must not be used
  bool set_intrinsic_param(const CameraIntrinsicParam & param);
  bool is_valid() const {return is_valid_;}

  double fx() const {return fx_;}
  double fy() const {return fy_;}
  double cx() const {return cx_;}
  double cy() const {return cy_;}

  // pixel -> undistorted normalized coordinate
  void lift_projective(
    const std::vector<Eigen::Vector2d> & pixels, std::vector<Eigen::Vector2d> & normals) const;

private:
  bool is_valid_{false};
  double fx_{0};
  double fy_{0};
  double cx_{0};
  double cy_{0};
  cv::Mat k_;
  cv::Mat d_;
};
}  // namespace slt_common
