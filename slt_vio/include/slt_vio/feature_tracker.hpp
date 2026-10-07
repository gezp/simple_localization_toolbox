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
#include <map>
#include <vector>

#include <opencv2/core.hpp>

#include "slt_common/camera_model/camera_model.hpp"
#include "yaml-cpp/yaml.h"

namespace slt_vio
{
// the tracking result of one output frame, the points are parallel arrays, un_pts are the
// undistorted normalized coordinates and velocity is expressed in normalized unit per second
struct TrackedFeatures
{
  double time = 0.0;
  std::vector<int> ids;
  std::vector<cv::Point2f> pts;
  std::vector<cv::Point2f> un_pts;
  std::vector<cv::Point2f> velocity;
  std::vector<int> track_cnt;
};

class FeatureTracker
{
public:
  // reads its keys from the `feature_tracker` section
  explicit FeatureTracker(const YAML::Node & config);
  // the intrinsics arrive on a topic after startup, it undistorts with them: set before the
  // first image
  void set_camera_model(const slt_common::CameraModel & camera_model);

  // an output frame additionally rejects outliers, refreshes the detection mask and detects
  // new features, a normal frame only propagates the existing ones
  void read_image(const cv::Mat & image, double time, bool output_frame);
  const TrackedFeatures & get_features() const {return features_;}

private:
  bool in_border(const cv::Point2f & pt) const;
  void set_mask();
  void detect_new_features();
  void reject_with_f();
  void undistort_points();
  void assign_new_ids();

  // drops the entries the tracking did not keep, the two vectors are walked in parallel
  static void reduce_vector(std::vector<cv::Point2f> & v, const std::vector<uchar> & status);
  static void reduce_vector(std::vector<int> & v, const std::vector<uchar> & status);
  static std::vector<Eigen::Vector2d> to_eigen_points(const std::vector<cv::Point2f> & pts);

private:
  // the pyr lk window and the pyramid depth are fixed: they are the values vins-mono uses
  static constexpr int kLkWinSize = 21;
  static constexpr int kLkMaxLevel = 3;
  // goodFeaturesToTrack quality level
  static constexpr double kQualityLevel = 0.01;
  // the detection border, a feature closer to the edge than this is dropped
  static constexpr int kBorderSize = 1;

  slt_common::CameraModel camera_model_;
  // config
  int max_cnt_{150};
  int min_dist_{25};
  double f_threshold_{1.0};
  double focal_length_{460.0};
  bool equalize_{false};

  // tracker state
  int next_id_{0};
  double time_{0};
  double prev_time_{0};
  cv::Mat cur_img_;
  cv::Mat forw_img_;
  cv::Mat mask_;
  std::vector<cv::Point2f> cur_pts_;
  std::vector<cv::Point2f> forw_pts_;
  std::vector<cv::Point2f> cur_un_pts_;
  std::vector<cv::Point2f> pts_velocity_;
  std::vector<int> ids_;
  std::vector<int> track_cnt_;
  std::map<int, cv::Point2f> prev_un_pts_map_;
  std::map<int, cv::Point2f> cur_un_pts_map_;
  TrackedFeatures features_;
};
}  // namespace slt_vio
