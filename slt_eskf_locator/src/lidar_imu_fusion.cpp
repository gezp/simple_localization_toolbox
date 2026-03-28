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

#include "slt_eskf_locator/lidar_imu_fusion.hpp"

#include <iostream>

#include "slt_common/sensor_data_utils.hpp"

namespace slt_eskf_locator
{

LidarImuFusion::LidarImuFusion() {}

bool LidarImuFusion::init_config(const YAML::Node & config_node)
{
  // gravity
  gravity_magnitude_ = config_node["earth"]["gravity_magnitude"].as<double>();
  // lidar pose noise
  auto pos_noise = config_node["measurement_noise"]["lidar_pose"]["pos"].as<double>();
  auto ori_noise = config_node["measurement_noise"]["lidar_pose"]["ori"].as<double>();
  lidar_pose_noise_ << pos_noise, pos_noise, pos_noise, ori_noise, ori_noise, ori_noise;
  // set up fusion method:
  auto fusion_method = config_node["fusion_method"].as<std::string>();
  if (fusion_method == "eskf") {
    kalman_filter_ = std::make_shared<Eskf>(config_node[fusion_method]);
  } else {
    std::cerr << "Fusion method " << fusion_method << " NOT FOUND!";
    return false;
  }
  std::cout << "Localization Fusion Method: " << fusion_method << std::endl;
  kalman_filter_->print_info();
  return true;
}

void LidarImuFusion::set_extrinsic(const Eigen::Matrix4d & T_base_imu) {T_base_imu_ = T_base_imu;}

bool LidarImuFusion::init_filter(
  const Eigen::Matrix4d & initial_pose, const Eigen::Vector3d & initial_velocity,
  const slt_common::ImuData & initial_imu)
{
  // init kalman filter
  slt_common::ImuNavState initial_nav_state;
  initial_nav_state.time = initial_imu.time;
  initial_nav_state.position = initial_pose.block<3, 1>(0, 3);
  initial_nav_state.orientation = initial_pose.block<3, 3>(0, 0);
  initial_nav_state.linear_velocity = initial_velocity;
  initial_nav_state.gravity = Eigen::Vector3d(0.0, 0.0, gravity_magnitude_);
  initial_nav_state.gyro_bias = Eigen::Vector3d::Zero();
  initial_nav_state.accel_bias = Eigen::Vector3d::Zero();
  kalman_filter_->init_state(initial_nav_state, initial_imu);
  current_imu_ = initial_imu;
  latest_correct_time_ = initial_imu.time;
  has_inited_ = true;
  std::cout << "Localization Init Succeeded at " << initial_imu.time << std::endl
            << "Init Position(IMU): " << initial_pose.block<3, 1>(0, 3).transpose() << std::endl
            << "Init Velocity(IMU): " << initial_velocity.transpose() << std::endl;
  return true;
}

bool LidarImuFusion::try_init_by_gnss()
{
  if (imu_buffer_.empty() || gnss_buffer_.empty()) {
    return false;
  }
  if (gnss_buffer_.front().time < imu_buffer_.front().time) {
    gnss_buffer_.pop_front();
    return false;
  }
  if (gnss_buffer_.front().time > imu_buffer_.back().time) {
    return false;
  }
  // odometry for imu
  auto odom = slt_common::transform_odom(gnss_buffer_.front(), T_base_imu_.inverse());
  Eigen::Matrix4d current_pose = odom.pose;
  Eigen::Vector3d current_vel = current_pose.block<3, 3>(0, 0) * odom.linear_velocity;
  gnss_buffer_.pop_front();
  // get sync imu
  slt_common::ImuData imu;
  while (imu_buffer_.front().time < odom.time) {
    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
  }
  auto & next_imu = imu_buffer_.front();
  auto current_imu = slt_common::interpolate_imu(imu, next_imu, odom.time);
  // init
  std::cout << "Init filter by GNSS odometry" << std::endl;
  init_filter(current_pose, current_vel, current_imu);
  return true;
}

bool LidarImuFusion::try_init_by_lidar()
{
  if (imu_buffer_.empty() || lidar_buffer_.size() < 2) {
    return false;
  }
  if (lidar_buffer_.front().time < imu_buffer_.front().time) {
    lidar_buffer_.pop_front();
    return false;
  }
  if (lidar_buffer_.front().time > imu_buffer_.back().time) {
    return false;
  }
  // pose and vel for imu
  auto current_time = lidar_buffer_.front().time;
  Eigen::Matrix4d current_pose = lidar_buffer_.at(0).pose * T_base_imu_;
  Eigen::Matrix4d next_pose = lidar_buffer_.at(1).pose * T_base_imu_;
  double dt = lidar_buffer_.at(1).time - lidar_buffer_.at(0).time;
  Eigen::Vector3d dx = next_pose.block<3, 1>(0, 3) - current_pose.block<3, 1>(0, 3);
  Eigen::Vector3d current_vel = dx / dt;
  lidar_buffer_.pop_front();
  // get sync imu
  slt_common::ImuData imu;
  while (imu_buffer_.front().time < current_time) {
    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
  }
  auto & next_imu = imu_buffer_.front();
  auto current_imu = slt_common::interpolate_imu(imu, next_imu, current_time);
  // init
  std::cout << "Init filter by Lidar Pose" << std::endl;
  init_filter(current_pose, current_vel, current_imu);
  return true;
}

bool LidarImuFusion::add_imu_data(const slt_common::ImuData & imu)
{
  imu_buffer_.push_back(imu);
  return true;
}

bool LidarImuFusion::add_lidar_data(const slt_common::OdomData & lidar_pose)
{
  lidar_buffer_.push_back(lidar_pose);
  return true;
}

bool LidarImuFusion::add_gnss_data(const slt_common::OdomData & gnss_pose)
{
  // only for initialization.
  // TODO(gezp) : use for observation
  if (has_inited_) {
    return true;
  }
  gnss_buffer_.push_back(gnss_pose);
  return true;
}

bool LidarImuFusion::process_imu_data(const slt_common::ImuData & imu)
{
  // check time
  if (imu.time < kalman_filter_->get_time()) {
    std::cout << "IMU has a older timestamp. Skip." << imu.time << std::endl;
    return false;
  }
  current_imu_ = imu;
  if (!kalman_filter_->predict(current_imu_)) {
    std::cout << "predict failed." << std::endl;
    return false;
  }
  return true;
}

bool LidarImuFusion::process_observation_data(const slt_common::OdomData & lidar_pose)
{
  // check time
  if (lidar_pose.time < kalman_filter_->get_time()) {
    std::cout << "Lidar Observation has a older timestamp. Skip."
              << lidar_pose.time - kalman_filter_->get_time() << std::endl;
    return false;
  }
  // predict if not sync
  if (lidar_pose.time != kalman_filter_->get_time()) {
    auto imu =
      slt_common::interpolate_imu(current_imu_, imu_buffer_.front(), lidar_pose.time);
    process_imu_data(imu);
  }
  // correct
  Eigen::Matrix4d pose = lidar_pose.pose * T_base_imu_;
  if (!kalman_filter_->observe_pose(pose, lidar_pose_noise_)) {
    std::cout << "correct failed." << std::endl;
    return false;
  }
  latest_correct_time_ = lidar_pose.time;
  return true;
}

bool LidarImuFusion::update()
{
  if (!has_inited_) {
    // try to init
    if (try_init_by_gnss()) {
      return true;
    }
    if (try_init_by_lidar()) {
      return true;
    }
    return false;
  }
  if (imu_buffer_.empty()) {
    return false;
  }
  if (lidar_buffer_.empty()) {
    // predict forward when no obsersation
    double predict_dt = latest_correct_time_ + 0.09;
    if (imu_buffer_.front().time > predict_dt) {
      // skip if over window
      return false;
    }
    auto imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    return process_imu_data(imu);
  } else {
    if (imu_buffer_.back().time < lidar_buffer_.front().time) {
      // skip if large imu delay, wait new imu data
      auto dt = lidar_buffer_.front().time - imu_buffer_.back().time;
      std::cout << "all imu is before lidar, large imu delay:" << dt << std::endl;
      return false;
    }
    if (imu_buffer_.front().time <= lidar_buffer_.front().time) {
      // predict forward when has obsersation
      auto imu = imu_buffer_.front();
      imu_buffer_.pop_front();
      return process_imu_data(imu);
    }
    // correct by obsersation
    auto current_lidar = lidar_buffer_.front();
    lidar_buffer_.pop_front();
    return process_observation_data(current_lidar);
  }
}

slt_common::ImuNavState LidarImuFusion::get_imu_nav_state()
{
  return kalman_filter_->get_imu_nav_state();
}

slt_common::OdomData LidarImuFusion::get_current_odom()
{
  auto nav_state = kalman_filter_->get_imu_nav_state();
  // odometry for imu frame
  slt_common::OdomData odom;
  odom.time = nav_state.time;
  odom.pose.block<3, 1>(0, 3) = nav_state.position;
  odom.pose.block<3, 3>(0, 0) = nav_state.orientation;
  odom.linear_velocity = nav_state.linear_velocity;
  odom.angular_velocity = current_imu_.angular_velocity;
  // odometry for base frame
  return slt_common::transform_odom(odom, T_base_imu_);
}

}  // namespace slt_eskf_locator
