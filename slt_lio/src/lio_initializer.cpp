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

#include "slt_lio/lio_initializer.hpp"

#include <cmath>
#include <iostream>
#include <memory>

#include "slt_common/lidar_utils.hpp"
#include "slt_common/point_cloud_registration/point_cloud_registration_factory.hpp"
#include "slt_common/sensor_data_utils.hpp"

namespace slt_lio
{

LioInitializer::LioInitializer(const YAML::Node & config)
{
  method_ = config["method"].as<std::string>();
  if (method_ != "static" && method_ != "dynamic" && method_ != "auto") {
    std::cerr << "LioInitializer: unknown init method " << method_ << ", use auto" << std::endl;
    method_ = "auto";
  }
  static_init_num_ = config["static_init_num"].as<int>();
  static_gyro_threshold_ = config["static_gyro_threshold"].as<double>();
  static_accel_threshold_ = config["static_accel_threshold"].as<double>();
  static_accel_noise_ = config["static_accel_noise"].as<double>();
  gravity_magnitude_ = std::abs(config["gravity_magnitude"].as<double>());
  if (method_ != "static") {
    registration_ = slt_common::PointCloudRegistrationFactory().create(config["registration"]);
  }
}

void LioInitializer::set_extrinsic(const Eigen::Matrix4d & T_imu_lidar)
{
  T_imu_lidar_ = T_imu_lidar;
}

bool LioInitializer::add_imu_data(const slt_common::ImuData & imu_data)
{
  imu_buffer_.push_back(imu_data);
  while (imu_buffer_.size() > kMaxImuBuffer) {
    imu_buffer_.pop_front();
  }
  return true;
}

bool LioInitializer::add_lidar_data(const slt_common::LidarData & lidar_data)
{
  lidar_buffer_.push_back(lidar_data);
  while (lidar_buffer_.size() > kNumInitScan) {
    lidar_buffer_.pop_front();
  }
  return true;
}

bool LioInitializer::try_init(slt_common::ImuNavState & state, slt_common::ImuData & imu_data)
{
  if (is_inited_) {
    return true;
  }
  // the state is built only from a full window, so the sensor's own startup (slow rate,
  // gaps) seeds it instead of being propagated
  if (static_cast<int>(imu_buffer_.size()) < static_init_num_) {
    return false;
  }
  bool ok = false;
  if (method_ == "static") {
    ok = init_static(state, imu_data);
  } else if (method_ == "dynamic") {
    ok = init_dynamic(state, imu_data);
  } else {
    ok = init_dynamic(state, imu_data);
    if (!ok) {
      ok = init_static(state, imu_data);
      if (ok) {
        std::cout << "LioInitializer: scan matching failed, fall back to the static "
          "initialization"
                  << std::endl;
      }
    }
  }
  if (!ok) {
    return false;
  }
  is_inited_ = true;
  return true;
}

void LioInitializer::reset()
{
  imu_buffer_.clear();
  lidar_buffer_.clear();
  last_registration_time_ = 0.0;
  is_inited_ = false;
}

bool LioInitializer::is_static(Eigen::Vector3d & gyro_avg, Eigen::Vector3d & accl_avg) const
{
  if (static_cast<int>(imu_buffer_.size()) < static_init_num_) {
    return false;
  }
  gyro_avg = Eigen::Vector3d::Zero();
  accl_avg = Eigen::Vector3d::Zero();
  for (int i = 0; i < static_init_num_; i++) {
    auto & imu = imu_buffer_[imu_buffer_.size() - 1 - i];
    gyro_avg += imu.angular_velocity;
    accl_avg += imu.linear_acceleration;
  }
  gyro_avg /= static_init_num_;
  accl_avg /= static_init_num_;
  // a vehicle driving straight and steady also has a small gyro: the accel variance
  // tells a window at rest from one that is not
  double accel_variance = 0.0;
  for (int i = 0; i < static_init_num_; i++) {
    auto & imu = imu_buffer_[imu_buffer_.size() - 1 - i];
    accel_variance += (imu.linear_acceleration - accl_avg).squaredNorm();
  }
  accel_variance = std::sqrt(accel_variance / static_init_num_);
  if (gyro_avg.norm() > static_gyro_threshold_) {
    return false;
  }
  if (std::abs(accl_avg.norm() - gravity_magnitude_) > static_accel_threshold_) {
    return false;
  }
  if (accel_variance > static_accel_noise_) {
    return false;
  }
  return true;
}

bool LioInitializer::init_static(slt_common::ImuNavState & state, slt_common::ImuData & imu_data)
{
  Eigen::Vector3d gyro_avg, accl_avg;
  if (!is_static(gyro_avg, accl_avg)) {
    return false;
  }
  imu_data = imu_buffer_.back();
  state.time = imu_data.time;
  // the accel bias is not observable at rest
  state.gyro_bias = gyro_avg;
  state.accel_bias = Eigen::Vector3d::Zero();
  // accel at rest points up in body frame, so it maps to +z of the world frame
  state.orientation =
    Eigen::Quaterniond::FromTwoVectors(accl_avg.normalized(), Eigen::Vector3d::UnitZ()).matrix();
  state.gravity = Eigen::Vector3d(0.0, 0.0, -gravity_magnitude_);
  state.linear_velocity = Eigen::Vector3d::Zero();
  std::cout << "LioInitializer: static initialization done, gyro bias " << gyro_avg.transpose()
            << std::endl;
  return true;
}

bool LioInitializer::init_dynamic(slt_common::ImuNavState & state, slt_common::ImuData & imu_data)
{
  slt_common::TwistData twist;
  if (!init_twist_by_lidar(twist)) {
    return false;
  }
  // state time: the imu tick nearest to the end of the scan the twist is from
  if (!find_nearest_imu(lidar_buffer_.back().time, imu_data)) {
    return false;
  }
  state.time = imu_data.time;
  // the bias is not observable while driving, the filter estimates them
  state.gyro_bias = Eigen::Vector3d::Zero();
  state.accel_bias = Eigen::Vector3d::Zero();
  // world = imu body frame at init, so the initial yaw is free
  state.orientation = Eigen::Matrix3d::Identity();
  // assumes the imu is roughly level at init
  state.gravity = Eigen::Vector3d(0.0, 0.0, -gravity_magnitude_);
  // lidar twist -> imu frame (rotation + lever arm)
  state.linear_velocity = slt_common::transform_twist(twist, T_imu_lidar_).linear_velocity;
  std::cout << "LioInitializer: dynamic initialization done, velocity(imu) "
            << state.linear_velocity.transpose() << std::endl;
  return true;
}

bool LioInitializer::find_nearest_imu(double time, slt_common::ImuData & imu_data) const
{
  bool found = false;
  double min_time_diff = kMaxImuTimeDiff;
  for (auto & data : imu_buffer_) {
    double time_diff = std::abs(data.time - time);
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      imu_data = data;
      found = true;
    }
  }
  return found;
}

bool LioInitializer::init_twist_by_lidar(slt_common::TwistData & twist)
{
  if (registration_ == nullptr || lidar_buffer_.size() < kNumInitScan) {
    return false;
  }
  // only retried when a newer scan arrives, this is called at imu rate
  if (lidar_buffer_.back().time <= last_registration_time_) {
    return false;
  }
  last_registration_time_ = lidar_buffer_.back().time;
  // the newest scan pair
  auto & target_data = lidar_buffer_[lidar_buffer_.size() - 2];
  auto & input_data = lidar_buffer_.back();
  double dt = input_data.time - target_data.time;
  if (dt < 1e-3) {
    return false;
  }
  registration_->set_target(slt_common::to_pointcloud_xyz(target_data.point_cloud));
  auto input = slt_common::to_pointcloud_xyz(input_data.point_cloud);
  if (!registration_->match(input, Eigen::Matrix4d::Identity())) {
    return false;
  }
  // the registration maps the input scan into the target (older) frame, the twist is
  // wanted in the input frame: rotate back by R^T
  Eigen::Matrix4d T_target_input = registration_->get_final_pose();
  Eigen::Matrix3d R_target_input = T_target_input.block<3, 3>(0, 0);
  twist.time = input_data.time;
  twist.linear_velocity = R_target_input.transpose() * T_target_input.block<3, 1>(0, 3) / dt;
  // T_target_input = exp([w_target]x * dt), w_input = R^T * w_target
  Eigen::AngleAxisd rotation(R_target_input);
  twist.angular_velocity = R_target_input.transpose() * (rotation.axis() * rotation.angle()) / dt;
  return true;
}

}  // namespace slt_lio
