// Copyright 2026 Zhenpeng Ge (https://github.com/zhenpengge).
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

#include "slt_lio/fastlio.hpp"

#include <pcl/common/transforms.h>

#include <iostream>

#include "slt_common/lidar_utils.hpp"

namespace slt_lio
{

Fastlio::Fastlio(const YAML::Node & config)
{
  config_ = config;
  eskf_ = std::make_unique<Eskf>(config["eskf"]);
  input_filter_ = std::make_shared<slt_common::VoxelFilter>(config["input_filter"]);
  display_filter_ = std::make_shared<slt_common::VoxelFilter>(config["display_filter"]);
  initializer_ = std::make_unique<LioInitializer>(config["lio_initializer"]);
  key_frame_distance_ = config["local_map"]["key_frame_distance"].as<double>();
  key_frame_angle_ = config["local_map"]["key_frame_angle"].as<double>();
  local_frame_num_ = config["local_map"]["local_frame_num"].as<int>();
  local_map_filter_ =
    std::make_shared<slt_common::VoxelFilter>(config["local_map"]["local_map_filter"]);
  correspondence_config_.num_nearby = config["correspondence_search"]["num_nearby"].as<int>();
  correspondence_config_.nearby_distance =
    config["correspondence_search"]["nearby_distance"].as<double>();
  correspondence_config_.plane_eigen_ratio =
    config["correspondence_search"]["plane_eigen_ratio"].as<double>();
  correspondence_config_.max_plane_distance =
    config["correspondence_search"]["max_plane_distance"].as<double>();
  bool enable = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enable);
  elapsed_time_statistics_.set_title("Fastlio");
  local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

Fastlio::~Fastlio()
{
}

void Fastlio::set_extrinsic(const Eigen::Matrix4d & T_imu_lidar, const Eigen::Matrix4d & T_base_imu)
{
  T_imu_lidar_ = T_imu_lidar;
  T_base_imu_ = T_base_imu;
  R_imu_lidar_ = T_imu_lidar_.block<3, 3>(0, 0);
  t_imu_lidar_ = T_imu_lidar_.block<3, 1>(0, 3);
  has_extrinsic_ = true;
  initializer_->set_extrinsic(T_imu_lidar_);
}

void Fastlio::add_imu_data(const slt_common::ImuData & imu_data)
{
  if (!is_inited_) {
    // the startup window seeds the state, never propagates: a gappy imu startup cannot
    // reach the filter
    initializer_->add_imu_data(imu_data);
    return;
  }
  imu_history_buffer_.emplace(imu_data.time, imu_data);
  while (imu_history_buffer_.size() > kMaxImuBuffer) {
    imu_history_buffer_.erase(imu_history_buffer_.begin());
  }
}

void Fastlio::add_lidar_data(const slt_common::LidarData & lidar_data)
{
  if (!is_inited_) {
    initializer_->add_lidar_data(lidar_data);
  }
  lidar_buffer_.push_back(lidar_data);
  while (lidar_buffer_.size() > kMaxLidarBuffer) {
    lidar_buffer_.pop_front();
  }
}

bool Fastlio::update()
{
  if (!has_extrinsic_) {
    return false;
  }
  if (!is_inited_ && !try_init()) {
    return false;
  }
  bool has_new_observation = false;
  // consume the oldest observation the history already covers
  while (!lidar_buffer_.empty()) {
    const auto & lidar_data = lidar_buffer_.front();
    const double t_obs = lidar_data.time + kScanDuration;
    // an observation older than the state does not describe it: drop it
    if (t_obs < eskf_->get_time()) {
      lidar_buffer_.pop_front();
      continue;
    }
    // the sample straddling the observation time is not here yet, wait for it
    if (imu_history_buffer_.empty() || imu_history_buffer_.rbegin()->first < t_obs) {
      break;
    }
    consume_observation(lidar_data, t_obs);
    lidar_buffer_.pop_front();
    has_new_observation = true;
  }
  return has_new_observation;
}

void Fastlio::consume_observation(const slt_common::LidarData & lidar_data, double t_obs)
{
  elapsed_time_statistics_.tic("consume_observation");
  // the state must describe the scan: the buffered imu carries the filter to the
  // observation time, the sample there lands it exactly on it
  propagate_eskf_to(t_obs);
  if (eskf_->get_time() < t_obs) {
    eskf_->predict(sample_imu(t_obs));
  }
  // the scan is registered as a snapshot of its time (no per-point time to deskew with)
  current_scan_ = slt_common::to_pointcloud_xyz(lidar_data.point_cloud);
  pcl::transformPointCloud(*current_scan_, *current_scan_, T_imu_lidar_.cast<float>());
  has_new_scan_ = true;
  observe_point_cloud();
  elapsed_time_statistics_.toc("consume_observation");
  elapsed_time_statistics_.print_all_info("consume_observation", 20);
}

void Fastlio::propagate_eskf_to(double time)
{
  for (auto it = imu_history_buffer_.upper_bound(eskf_->get_time());
    it != imu_history_buffer_.end() && it->first <= time; ++it)
  {
    eskf_->predict(it->second);
  }
}

slt_common::ImuData Fastlio::sample_imu(double time)
{
  auto right = imu_history_buffer_.lower_bound(time);
  if (right == imu_history_buffer_.begin() || right == imu_history_buffer_.end()) {
    // `time` falls outside the history (imu dropout, or a backlog trimmed past it): the
    // nearest sample is all that is left, keep `time` so the caller's state lands on it
    slt_common::ImuData imu =
      right == imu_history_buffer_.end() ? std::prev(right)->second : right->second;
    imu.time = time;
    return imu;
  }
  return slt_common::interpolate_imu(std::prev(right)->second, right->second, time);
}

bool Fastlio::try_init()
{
  // the seed imu sample is the one the state is defined at (state.time == imu_data.time)
  slt_common::ImuNavState state;
  slt_common::ImuData imu_data;
  if (!initializer_->try_init(state, imu_data)) {
    return false;
  }
  eskf_->init_state(state, imu_data);
  initializer_->reset();
  while (!lidar_buffer_.empty() && lidar_buffer_.front().time <= eskf_->get_time()) {
    lidar_buffer_.pop_front();
  }
  is_inited_ = true;
  std::cout << "Fastlio: initialization done" << std::endl;
  return true;
}

bool Fastlio::is_initialized() const
{
  return is_inited_;
}

slt_common::ImuNavState Fastlio::get_imu_nav_state()
{
  return eskf_->get_imu_nav_state();
}

slt_common::ImuData Fastlio::get_imu_data()
{
  return eskf_->get_imu_data();
}

Eigen::Matrix4d Fastlio::get_current_pose()
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  const auto & state = eskf_->get_imu_nav_state();
  pose.block<3, 3>(0, 0) = state.orientation;
  pose.block<3, 1>(0, 3) = state.position;
  return pose;
}

Fastlio::PointCloudPtr Fastlio::get_current_scan()
{
  if (!has_new_scan_ || current_scan_ == nullptr) {
    return nullptr;
  }
  has_new_scan_ = false;
  auto scan = display_filter_->apply(current_scan_);
  pcl::transformPointCloud(*scan, *scan, get_current_pose());
  return scan;
}

bool Fastlio::observe_point_cloud()
{
  const auto & state = eskf_->get_imu_nav_state();
  // two-pass update: match all correspondences then observe once
  auto input = input_filter_->apply(current_scan_);
  auto observations = PointPlaneCorrespondenceSearch::search(
    input, local_map_, kd_tree_, state.orientation, state.position, correspondence_config_);
  if (!observations.empty()) {
    eskf_->observe_point_cloud(observations);
  }
  if (key_frames_.empty() || check_new_key_frame()) {
    update_local_map();
  }
  return true;
}

Fastlio::PointCloudPtr Fastlio::get_local_map()
{
  if (!has_new_local_map_) {
    return nullptr;
  }
  has_new_local_map_ = false;
  return local_map_;
}

bool Fastlio::check_new_key_frame()
{
  auto current_pose = get_current_pose();
  Eigen::Vector3d dis = last_key_frame_pose_.block<3, 1>(0, 3) - current_pose.block<3, 1>(0, 3);
  if (dis.norm() > key_frame_distance_) {
    return true;
  }
  Eigen::Matrix3d R_rel =
    last_key_frame_pose_.block<3, 3>(0, 0).transpose() * current_pose.block<3, 3>(0, 0);
  if (Eigen::AngleAxisd(R_rel).angle() > key_frame_angle_ * M_PI / 180.0) {
    return true;
  }
  return false;
}

bool Fastlio::update_local_map()
{
  auto current_pose = get_current_pose();
  Frame frame;
  auto scan = current_scan_->makeShared();
  Eigen::Affine3f T_world_imu = Eigen::Translation3f(current_pose.block<3, 1>(0, 3).cast<float>()) *
    current_pose.block<3, 3>(0, 0).cast<float>();
  pcl::transformPointCloud(*scan, *scan, T_world_imu);
  frame.point_cloud = scan;
  key_frames_.push_back(frame);
  while (key_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    key_frames_.pop_front();
  }
  last_key_frame_pose_ = current_pose;
  local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto & key_frame : key_frames_) {
    *local_map_ += *key_frame.point_cloud;
  }
  local_map_ = local_map_filter_->apply(local_map_);
  if (!local_map_->empty()) {
    kd_tree_.setInputCloud(local_map_);
  }
  has_new_local_map_ = true;
  return true;
}

}  // namespace slt_lio
