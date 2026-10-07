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

#include "slt_vio/feature_tracker.hpp"

#include <algorithm>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace slt_vio
{
void FeatureTracker::reduce_vector(std::vector<cv::Point2f> & v, const std::vector<uchar> & status)
{
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); i++) {
    if (status[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

void FeatureTracker::reduce_vector(std::vector<int> & v, const std::vector<uchar> & status)
{
  int j = 0;
  for (int i = 0; i < static_cast<int>(v.size()); i++) {
    if (status[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

std::vector<Eigen::Vector2d> FeatureTracker::to_eigen_points(const std::vector<cv::Point2f> & pts)
{
  std::vector<Eigen::Vector2d> pixels;
  pixels.reserve(pts.size());
  for (const auto & pt : pts) {
    pixels.emplace_back(pt.x, pt.y);
  }
  return pixels;
}

FeatureTracker::FeatureTracker(const YAML::Node & config)
{
  if (config["max_cnt"]) {
    max_cnt_ = config["max_cnt"].as<int>();
  }
  if (config["min_dist"]) {
    min_dist_ = config["min_dist"].as<int>();
  }
  if (config["f_threshold"]) {
    f_threshold_ = config["f_threshold"].as<double>();
  }
  if (config["focal_length"]) {
    focal_length_ = config["focal_length"].as<double>();
  }
  if (config["equalize"]) {
    equalize_ = config["equalize"].as<bool>();
  }
}

void FeatureTracker::set_camera_model(const slt_common::CameraModel & camera_model)
{
  camera_model_ = camera_model;
}

bool FeatureTracker::in_border(const cv::Point2f & pt) const
{
  const int img_x = cvRound(pt.x);
  const int img_y = cvRound(pt.y);
  return kBorderSize <= img_x && img_x < forw_img_.cols - kBorderSize && kBorderSize <= img_y &&
         img_y < forw_img_.rows - kBorderSize;
}

void FeatureTracker::read_image(const cv::Mat & image, double time, bool output_frame)
{
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  cv::Mat img;
  if (equalize_) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, img);
  } else {
    img = gray;
  }
  time_ = time;
  if (forw_img_.empty()) {
    cur_img_ = forw_img_ = img;
  } else {
    forw_img_ = img;
  }
  forw_pts_.clear();
  if (cur_pts_.size() > 0) {
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
      cur_img_, forw_img_, cur_pts_, forw_pts_, status, err, cv::Size(kLkWinSize, kLkWinSize),
      kLkMaxLevel);
    for (int i = 0; i < static_cast<int>(forw_pts_.size()); i++) {
      if (status[i] && !in_border(forw_pts_[i])) {
        status[i] = 0;
      }
    }
    reduce_vector(cur_pts_, status);
    reduce_vector(forw_pts_, status);
    reduce_vector(ids_, status);
    reduce_vector(cur_un_pts_, status);
    reduce_vector(track_cnt_, status);
  }
  for (auto & cnt : track_cnt_) {
    cnt++;
  }
  if (output_frame) {
    reject_with_f();
    set_mask();
    detect_new_features();
  }
  cur_img_ = forw_img_;
  cur_pts_ = forw_pts_;
  undistort_points();
  // vins-mono assigns the ids after the undistortion, the map keys of the brand new points
  // are therefore not in prev_un_pts_map_ and their first velocity reads as zero
  assign_new_ids();
  prev_time_ = time_;

  features_.time = time_;
  features_.ids = ids_;
  features_.pts = cur_pts_;
  features_.un_pts = cur_un_pts_;
  features_.velocity = pts_velocity_;
  features_.track_cnt = track_cnt_;
}

void FeatureTracker::reject_with_f()
{
  if (forw_pts_.size() < 8) {
    return;
  }
  // the normalized points are reprojected with a virtual focal length, the fundamental
  // matrix ransac only needs a consistent pixel-ish scale to apply f_threshold_
  std::vector<Eigen::Vector2d> cur_norm;
  std::vector<Eigen::Vector2d> forw_norm;
  camera_model_.lift_projective(to_eigen_points(cur_pts_), cur_norm);
  camera_model_.lift_projective(to_eigen_points(forw_pts_), forw_norm);
  if (cur_norm.size() != cur_pts_.size() || forw_norm.size() != forw_pts_.size()) {
    return;
  }
  const double half_col = cur_img_.cols / 2.0;
  const double half_row = cur_img_.rows / 2.0;
  std::vector<cv::Point2f> un_cur_pts(cur_pts_.size());
  std::vector<cv::Point2f> un_forw_pts(forw_pts_.size());
  for (size_t i = 0; i < cur_pts_.size(); i++) {
    un_cur_pts[i] = cv::Point2f(
      focal_length_ * cur_norm[i].x() + half_col, focal_length_ * cur_norm[i].y() + half_row);
  }
  for (size_t i = 0; i < forw_pts_.size(); i++) {
    un_forw_pts[i] = cv::Point2f(
      focal_length_ * forw_norm[i].x() + half_col, focal_length_ * forw_norm[i].y() + half_row);
  }
  std::vector<uchar> status;
  cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, f_threshold_, 0.99, status);
  reduce_vector(cur_pts_, status);
  reduce_vector(forw_pts_, status);
  reduce_vector(cur_un_pts_, status);
  reduce_vector(ids_, status);
  reduce_vector(track_cnt_, status);
}

void FeatureTracker::set_mask()
{
  mask_ = cv::Mat(forw_img_.rows, forw_img_.cols, CV_8UC1, cv::Scalar(255));
  // prefer to keep features that are tracked for long time
  std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
  cnt_pts_id.reserve(forw_pts_.size());
  for (size_t i = 0; i < forw_pts_.size(); i++) {
    cnt_pts_id.push_back(std::make_pair(track_cnt_[i], std::make_pair(forw_pts_[i], ids_[i])));
  }
  std::sort(
    cnt_pts_id.begin(), cnt_pts_id.end(),
    [](const std::pair<int, std::pair<cv::Point2f, int>> & a,
    const std::pair<int, std::pair<cv::Point2f, int>> & b) {return a.first > b.first;});
  forw_pts_.clear();
  ids_.clear();
  track_cnt_.clear();
  for (auto & it : cnt_pts_id) {
    if (mask_.at<uchar>(it.second.first) == 255) {
      forw_pts_.push_back(it.second.first);
      ids_.push_back(it.second.second);
      track_cnt_.push_back(it.first);
      cv::circle(mask_, it.second.first, min_dist_, 0, -1);
    }
  }
}

void FeatureTracker::detect_new_features()
{
  const int n_max_cnt = max_cnt_ - static_cast<int>(forw_pts_.size());
  if (n_max_cnt <= 0) {
    return;
  }
  std::vector<cv::Point2f> n_pts;
  cv::goodFeaturesToTrack(
    forw_img_, n_pts, n_max_cnt, kQualityLevel, min_dist_, mask_);
  for (auto & pt : n_pts) {
    forw_pts_.push_back(pt);
    ids_.push_back(-1);
    track_cnt_.push_back(1);
  }
}

void FeatureTracker::assign_new_ids()
{
  for (auto & id : ids_) {
    if (id == -1) {
      id = next_id_++;
    }
  }
}

void FeatureTracker::undistort_points()
{
  cur_un_pts_.clear();
  cur_un_pts_map_.clear();
  std::vector<Eigen::Vector2d> normals;
  camera_model_.lift_projective(to_eigen_points(cur_pts_), normals);
  if (normals.size() != cur_pts_.size()) {
    return;
  }
  for (size_t i = 0; i < cur_pts_.size(); i++) {
    cur_un_pts_.push_back(cv::Point2f(normals[i].x(), normals[i].y()));
    cur_un_pts_map_.insert(
      std::make_pair(ids_[i], cv::Point2f(normals[i].x(), normals[i].y())));
  }
  if (!prev_un_pts_map_.empty()) {
    const double dt = time_ - prev_time_;
    pts_velocity_.clear();
    for (size_t i = 0; i < cur_un_pts_.size(); i++) {
      auto it = ids_[i] != -1 ? prev_un_pts_map_.find(ids_[i]) : prev_un_pts_map_.end();
      if (it != prev_un_pts_map_.end() && dt > 0) {
        pts_velocity_.push_back(
          cv::Point2f(
            (cur_un_pts_[i].x - it->second.x) / dt, (cur_un_pts_[i].y - it->second.y) / dt));
      } else {
        pts_velocity_.push_back(cv::Point2f(0, 0));
      }
    }
  } else {
    pts_velocity_.assign(cur_pts_.size(), cv::Point2f(0, 0));
  }
  prev_un_pts_map_ = cur_un_pts_map_;
}

}  // namespace slt_vio
