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

#include "slt_common/camera_model/camera_model.hpp"

#include <algorithm>
#include <cstddef>

#include <opencv2/calib3d.hpp>

namespace slt_common
{

bool CameraModel::set_intrinsic_param(const CameraIntrinsicParam & param)
{
  is_valid_ = false;
  if (param.intrinsics.size() < 4) {
    return false;
  }
  // an empty distortion_model is accepted as a plain pinhole
  if (!param.distortion_model.empty() && param.distortion_model != "plumb_bob") {
    return false;
  }
  const double fx = param.intrinsics[0];
  const double fy = param.intrinsics[1];
  if (fx <= 0 || fy <= 0) {
    return false;
  }
  fx_ = fx;
  fy_ = fy;
  cx_ = param.intrinsics[2];
  cy_ = param.intrinsics[3];
  k_ = cv::Mat::eye(3, 3, CV_64F);
  k_.at<double>(0, 0) = fx_;
  k_.at<double>(1, 1) = fy_;
  k_.at<double>(0, 2) = cx_;
  k_.at<double>(1, 2) = cy_;
  // plumb_bob uses up to 5 coeffs; cv::undistortPoints accepts only 0/1/4/5/8/12/14
  const size_t num = std::min<size_t>(param.distortion_coeffs.size(), 5);
  d_ = cv::Mat::zeros(1, static_cast<int>(num), CV_64F);
  for (size_t i = 0; i < num; i++) {
    d_.at<double>(0, static_cast<int>(i)) = param.distortion_coeffs[i];
  }
  is_valid_ = true;
  return true;
}

void CameraModel::lift_projective(
  const std::vector<Eigen::Vector2d> & pixels, std::vector<Eigen::Vector2d> & normals) const
{
  normals.clear();
  if (!is_valid_ || pixels.empty()) {
    return;
  }
  std::vector<cv::Point2d> src;
  src.reserve(pixels.size());
  for (const auto & pixel : pixels) {
    src.emplace_back(pixel.x(), pixel.y());
  }
  // passing no P yields undistorted normalized coordinates
  std::vector<cv::Point2d> dst;
  cv::undistortPoints(src, dst, k_, d_);
  normals.reserve(dst.size());
  for (const auto & point : dst) {
    normals.emplace_back(point.x, point.y);
  }
}

}  // namespace slt_common
