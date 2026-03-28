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

#include "slt_graph_locator/sliding_window.hpp"

#include <chrono>

#include "slt_common/sensor_data_utils.hpp"

namespace slt_graph_locator
{

SlidingWindow::SlidingWindow() {}

SlidingWindow::~SlidingWindow() {}

bool SlidingWindow::init_with_config(const YAML::Node & config_node)
{
  // key frame selection config
  key_frame_config_.max_distance = config_node["key_frame"]["max_distance"].as<float>();
  key_frame_config_.max_interval = config_node["key_frame"]["max_interval"].as<float>();
  // constraints
  use_gnss_ = config_node["use_gnss"].as<bool>();
  use_lidar_pose_ = config_node["use_lidar_pose"].as<bool>();
  use_imu_pre_integration_ = config_node["use_imu_pre_integration"].as<bool>();
  sliding_window_size_ = config_node["sliding_window_size"].as<int>();
  // gravity
  gravity_magnitude_ = config_node["earth"]["gravity_magnitude"].as<double>();
  imu_config_.gravity = Eigen::Vector3d(0.0, 0.0, -gravity_magnitude_);
  // measurement noise
  lidar_odometry_noise_.resize(6);
  lidar_pose_noise_.resize(6);
  for (int i = 0; i < 6; ++i) {
    lidar_odometry_noise_(i) = config_node["measurement_noise"]["lidar_odometry"][i].as<double>();
    lidar_pose_noise_(i) = config_node["measurement_noise"]["lidar_pose"][i].as<double>();
  }
  gnss_position_noise_.resize(3);
  for (int i = 0; i < 3; i++) {
    gnss_position_noise_(i) = config_node["measurement_noise"]["gnss_position"][i].as<double>();
  }
  //
  imu_integration_ = std::make_shared<slt_imu_odometry::ImuIntegration>();
  // imu noise
  imu_config_.accel_noise = config_node["imu_noise"]["accel"].as<double>();
  imu_config_.gyro_noise = config_node["imu_noise"]["gyro"].as<double>();
  imu_config_.accel_bias_noise = config_node["imu_noise"]["accel_bias"].as<double>();
  imu_config_.gyro_bias_noise = config_node["imu_noise"]["gyro_bias"].as<double>();
  // c. sliding window config:
  init_graph_optimizer(config_node);
  gnss_odom_buffer_ = std::make_shared<slt_common::OdomDataBuffer>(10000);
  // print info
  std::cout << "SlidingWindow params:" << std::endl
            << "\timu_noise.accel: " << imu_config_.accel_noise << std::endl
            << "\timu_noise.gyro: " << imu_config_.gyro_noise << std::endl
            << "\timu_noise.accel_bias: " << imu_config_.accel_bias_noise << std::endl
            << "\timu_noise.gyro_bias: " << imu_config_.gyro_bias_noise << std::endl
            << std::endl;
  return true;
}

void SlidingWindow::set_extrinsic(const Eigen::Matrix4d & T_base_imu) {T_base_imu_ = T_base_imu;}

bool SlidingWindow::init_graph_optimizer(const YAML::Node & /*config_node*/)
{
  graph_optimizer_ = std::make_shared<CeresGraphOptimizer>(imu_config_);
  return true;
}

bool SlidingWindow::add_imu_data(const slt_common::ImuData & imu)
{
  if (!imu_buffer_.empty() && imu.time <= imu_buffer_.back().time) {
    std::cout << "receive a early imu data" << std::endl;
    return false;
  }
  imu_buffer_.push_back(imu);
  if (imu_buffer_.size() > 1000) {
    std::cout << "too many old imu in buffer, drop imu data" << std::endl;
    imu_buffer_.pop_front();
  }
  return true;
}

bool SlidingWindow::add_lidar_pose(const slt_common::OdomData & lidar_pose)
{
  if (!lidar_pose_buffer_.empty() && lidar_pose.time <= lidar_pose_buffer_.back().time) {
    std::cout << "receive a early lidar data" << std::endl;
    return false;
  }
  lidar_pose_buffer_.push_back(lidar_pose);
  // at least keep one history
  if (lidar_pose_buffer_.size() > 100 && unhandled_lidar_idx_ > 1) {
    lidar_pose_buffer_.pop_front();
    unhandled_lidar_idx_--;
  }
  return true;
}

bool SlidingWindow::add_gnss_pose(const slt_common::OdomData & gnss_pose)
{
  gnss_odom_buffer_->add_data(gnss_pose);
  return true;
}

bool SlidingWindow::update()
{
  // initialization
  if (!has_inited_) {
    // init graph
    if (!try_init_from_lidar()) {
      return false;
    }
    has_inited_ = true;
    return true;
  }
  // process lidar
  if (check_valid_lidar()) {
    // create Key frame
    KeyFrame key_frame;
    // get lidar pose
    auto & lidar_pose = lidar_pose_buffer_.at(unhandled_lidar_idx_);
    unhandled_lidar_idx_++;
    key_frame.lidar_pose = lidar_pose;
    key_frame.last_lidar_pose = lidar_pose_buffer_.at(unhandled_lidar_idx_ - 2);
    latest_key_pose_ = lidar_pose;
    // get synced integrated_imu_buffer
    key_frame.pre_integration_buffer.clear();
    key_frame.pre_integration_buffer.push_back(lastest_key_imu_);
    get_synced_imu_buffer(lidar_pose.time, key_frame.pre_integration_buffer);
    lastest_key_imu_ = key_frame.pre_integration_buffer.back();
    // get synced gnss pose
    slt_common::OdomData synced_gnss_pose;
    if (gnss_odom_buffer_->get_interpolated_data(lidar_pose.time, synced_gnss_pose)) {
      key_frame.gnss_pose = synced_gnss_pose;
      key_frame.has_valid_gnss = true;
    } else {
      key_frame.has_valid_gnss = false;
    }
    // update graph
    update_graph(key_frame);
    // get current imu data and nav state
    current_imu_ = lastest_key_imu_;
    current_imu_nav_state_ = graph_optimizer_->get_imu_nav_state();
    imu_integration_->reset(current_imu_nav_state_, true);
    imu_integration_->integrate(current_imu_);
    unhandled_imu_idx_ = 0;
    return true;
  }
  // process imu
  if (unhandled_imu_idx_ < imu_buffer_.size()) {
    double predict_dt = lastest_key_imu_.time + 0.09;
    if (imu_buffer_.at(unhandled_imu_idx_).time > predict_dt) {
      // skip if over window
      return false;
    }
    current_imu_ = imu_buffer_.at(unhandled_imu_idx_);
    imu_integration_->integrate(current_imu_);
    current_imu_nav_state_ = imu_integration_->get_imu_nav_state();
    unhandled_imu_idx_++;
    return true;
  }
  return false;
}

slt_common::ImuNavState SlidingWindow::get_imu_nav_state()
{
  return current_imu_nav_state_;
}

slt_common::OdomData SlidingWindow::get_current_odom()
{
  // odometry for imu frame
  slt_common::OdomData odom;
  odom.time = current_imu_nav_state_.time;
  odom.pose.block<3, 1>(0, 3) = current_imu_nav_state_.position;
  odom.pose.block<3, 3>(0, 0) = current_imu_nav_state_.orientation;
  odom.linear_velocity = current_imu_nav_state_.linear_velocity;
  odom.angular_velocity = current_imu_.angular_velocity;
  // odometry for base frame
  return slt_common::transform_odom(odom, T_base_imu_);
}

bool SlidingWindow::check_valid_lidar()
{
  if (lidar_pose_buffer_.size() < unhandled_lidar_idx_ + 1) {
    return false;
  }
  auto & lidar_pose = lidar_pose_buffer_.at(unhandled_lidar_idx_);
  if (imu_buffer_.back().time < lidar_pose.time) {
    // large imu delay. wait imu
    std::cout << "large imu delay" << std::endl;
    return false;
  }
  if (imu_buffer_.front().time > lidar_pose.time) {
    // drop old lidar data
    unhandled_lidar_idx_++;
    std::cout << "old lidar data" << imu_buffer_.front().time - lidar_pose.time << std::endl;
    return false;
  }
  if (has_inited_ && !check_new_key_frame(lidar_pose)) {
    unhandled_lidar_idx_++;
    // check new key frame
    return false;
  }
  return true;
}

bool SlidingWindow::check_new_key_frame(const slt_common::OdomData & odom)
{
  Eigen::Vector3d dx = odom.pose.block<3, 1>(0, 3) - latest_key_pose_.pose.block<3, 1>(0, 3);
  if (dx.lpNorm<1>() > key_frame_config_.max_distance) {
    return true;
  }
  if ((odom.time - latest_key_pose_.time) > key_frame_config_.max_interval) {
    return true;
  }
  return false;
}

int SlidingWindow::create_graph_node(
  double time, const Eigen::Matrix4d & pose, const Eigen::Vector3d & vel)
{
  slt_common::ImuNavState imu_nav_state;
  imu_nav_state.time = time;
  imu_nav_state.position = pose.block<3, 1>(0, 3);
  imu_nav_state.orientation = pose.block<3, 3>(0, 0);
  imu_nav_state.linear_velocity = vel;
  if (graph_optimizer_->get_vertex_count() > 0) {
    auto last_state = graph_optimizer_->get_imu_nav_state();
    imu_nav_state.accel_bias = last_state.accel_bias;
    imu_nav_state.gyro_bias = last_state.gyro_bias;
  }
  // add vertex
  int vertex_idx = graph_optimizer_->add_vertex(imu_nav_state, false);
  return vertex_idx;
}

int SlidingWindow::create_graph_node_from_odom(const slt_common::OdomData & odom)
{
  auto odom_imu = transform_odom(odom, T_base_imu_.inverse());
  Eigen::Vector3d vel = odom_imu.pose.block<3, 3>(0, 0) * odom_imu.linear_velocity;
  return create_graph_node(odom_imu.time, odom_imu.pose, vel);
}

int SlidingWindow::create_graph_node_from_lidar(
  const slt_common::OdomData & pose,
  const slt_common::OdomData & neighbor_pose)
{
  // get approximate average velocity between current pose and neighbor pose
  Eigen::Matrix4d pose_imu = pose.pose * T_base_imu_;
  Eigen::Matrix4d neighbor_pose_imu = neighbor_pose.pose * T_base_imu_;
  Eigen::Vector3d dx = neighbor_pose_imu.block<3, 1>(0, 3) - pose_imu.block<3, 1>(0, 3);
  Eigen::Vector3d current_vel = dx / (neighbor_pose.time - pose.time);
  return create_graph_node(pose.time, pose_imu, current_vel);
}

int SlidingWindow::create_graph_node_from_imu(
  const std::vector<slt_common::ImuData> & imus)
{
  auto last_state = graph_optimizer_->get_imu_nav_state();
  slt_imu_odometry::ImuIntegration integration;
  integration.reset(last_state);
  for (auto & imu : imus) {
    integration.integrate(imu);
  }
  // add vertex
  int vertex_idx = graph_optimizer_->add_vertex(integration.get_imu_nav_state(), false);
  return vertex_idx;
}

bool SlidingWindow::get_synced_imu_buffer(
  double time, std::vector<slt_common::ImuData> & buffer)
{
  if (imu_buffer_.empty() || imu_buffer_.back().time < time) {
    return false;
  }
  slt_common::ImuData imu;
  while (imu_buffer_.front().time <= time) {
    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    buffer.push_back(imu);
  }
  // get sync imu
  if (imu.time != time) {
    imu = slt_common::interpolate_imu(imu, imu_buffer_.front(), time);
    buffer.push_back(imu);
  }
  return true;
}

bool SlidingWindow::try_init_from_lidar()
{
  if (lidar_pose_buffer_.size() < unhandled_lidar_idx_ + 2) {
    return false;
  }
  if (!check_valid_lidar()) {
    return false;
  }
  // get current lidar
  auto lidar_pose = lidar_pose_buffer_.at(unhandled_lidar_idx_);
  auto next_pose = lidar_pose_buffer_.at(unhandled_lidar_idx_ + 1);
  unhandled_lidar_idx_++;
  latest_key_pose_ = lidar_pose;
  // get current imu
  std::vector<slt_common::ImuData> synced_imu_buffer;
  get_synced_imu_buffer(lidar_pose.time, synced_imu_buffer);
  lastest_key_imu_ = synced_imu_buffer.back();
  // create graph node from lidar
  int vertex_idx = create_graph_node_from_lidar(lidar_pose, next_pose);
  // add lidar pose constraint into graph
  if (use_lidar_pose_) {
    Eigen::Matrix4d prior_pose = lidar_pose.pose * T_base_imu_;
    graph_optimizer_->add_absolute_pose_edge(vertex_idx, prior_pose, lidar_pose_noise_);
  }
  // get current imu data and nav state
  current_imu_ = lastest_key_imu_;
  current_imu_nav_state_ = graph_optimizer_->get_imu_nav_state();
  imu_integration_->reset(current_imu_nav_state_, true);
  imu_integration_->integrate(current_imu_);
  unhandled_imu_idx_ = 0;
  std::cout << "try_init_from_lidar successfully!" << std::endl;
  return true;
}

bool SlidingWindow::update_graph(const KeyFrame & key_frame)
{
  // create graph node from lidar
  // int vertex_idx = create_graph_node_from_lidar(key_frame.lidar_pose,
  //                                               key_frame.last_lidar_pose);
  // create graph node from imu
  int vertex_idx = create_graph_node_from_imu(key_frame.pre_integration_buffer);
  // add lidar pose constraint into graph
  if (use_lidar_pose_) {
    Eigen::Matrix4d prior_pose = key_frame.lidar_pose.pose * T_base_imu_;
    graph_optimizer_->add_absolute_pose_edge(vertex_idx, prior_pose, lidar_pose_noise_);
  }
  // add gnss position constraint into graph
  if (use_gnss_ && key_frame.has_valid_gnss) {
    // TODO(gezp)
  }
  // add imu pre integration constraint into graph
  if (use_imu_pre_integration_) {
    std::cout << "pre integration buffer size:"
              << key_frame.pre_integration_buffer.size() << std::endl;
    graph_optimizer_->add_imu_pre_integration_edge(
      vertex_idx - 1, vertex_idx, key_frame.pre_integration_buffer);
  }
  // optimize
  std::cout << "------ Graph optimazation " << std::endl;
  auto start = std::chrono::steady_clock::now();
  bool ok = graph_optimizer_->optimize();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used = end - start;
  auto nav_state = graph_optimizer_->get_imu_nav_state();
  // print info
  std::cout << "Time Used: " << time_used.count() << " seconds." << std::endl
            << "Imu bias: ba " << nav_state.accel_bias.transpose()
            << ", bg " << nav_state.gyro_bias.transpose() << std::endl
            << "------" << std::endl;
  if (!ok) {
    return false;
  }
  // marginalize
  if (graph_optimizer_->get_vertex_count() > sliding_window_size_) {
    graph_optimizer_->marginalize(1);
  }
  return true;
}

}  // namespace slt_graph_locator
