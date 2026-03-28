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

#include <cmath>
#include <iostream>
#include <sophus/so3.hpp>

#include "slt_eskf_locator/kalman_filter/eskf.hpp"

namespace slt_eskf_locator
{

Eskf::Eskf(const YAML::Node & node)
{
  // prior and process noise covariance:
  prior_noise_ = node["covariance"]["prior"].as<double>();
  gyro_noise_ = node["covariance"]["gyro"].as<double>();
  accel_noise_ = node["covariance"]["accel"].as<double>();
  gyro_bias_noise_ = node["covariance"]["gyro_bias"].as<double>();
  accel_bias_noise_ = node["covariance"]["accel_bias"].as<double>();
  // reset prior state & covariance:
  X_.setZero();
  P_ = prior_noise_ * Eigen::Matrix<double, kDimState, kDimState>::Identity();
  // process noise:
  Q_.setZero();
  Q_.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) = accel_noise_ * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) = gyro_noise_ * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) =
    accel_bias_noise_ * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) =
    gyro_bias_noise_ * Eigen::Matrix3d::Identity();
  // imu integration
  imu_integration_ = std::make_shared<ImuIntegration>();
}

void Eskf::print_info()
{
  std::cout << "eskf params:" << std::endl
            << "\tprior noise: " << prior_noise_ << std::endl
            << "\tprocess noise gyro: " << gyro_noise_ << std::endl
            << "\tprocess noise accel: " << accel_noise_ << std::endl
            << "\tprocess noise gyro_bias: " << gyro_bias_noise_ << std::endl
            << "\tprocess noise accel_bias: " << accel_bias_noise_ << std::endl
            << std::endl;
}

void Eskf::init_state(
  const slt_common::ImuNavState & nav_state, const slt_common::ImuData & imu_data)
{
  time_ = nav_state.time;
  pos_ = nav_state.position;
  ori_ = nav_state.orientation;
  vel_ = nav_state.linear_velocity;
  gravity_ = nav_state.gravity;
  accl_bias_ = nav_state.accel_bias;
  gyro_bias_ = nav_state.gyro_bias;
  // init imu integration
  imu_integration_->init(nav_state, imu_data);
}

bool Eskf::predict(const slt_common::ImuData & imu_data)
{
  // check
  if (imu_data.time < time_) {
    return false;
  }
  double dt = imu_data.time - time_;
  time_ = imu_data.time;
  // imu integration
  imu_integration_->integrate(imu_data);
  auto state = imu_integration_->get_state();
  pos_ = state.position;
  ori_ = state.orientation;
  vel_ = state.linear_velocity;
  // update process equation
  Eigen::Matrix3d R_wb = ori_;
  Eigen::Vector3d w_b = imu_data.angular_velocity;
  Eigen::Vector3d a_b = imu_data.linear_acceleration;
  F_.setZero();
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Eigen::Matrix3d::Identity();
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) = -R_wb * Sophus::SO3d::hat(a_b);
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -R_wb;
  F_.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel) = R_wb;
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = -Sophus::SO3d::hat(w_b);
  F_ = Eigen::Matrix<double, kDimState, kDimState>::Identity() + F_ * dt;
  B_.setZero();
  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseAccel) = R_wb * dt;
  B_.block<3, 3>(kIndexErrorVel, kIndexNoiseGyro) = Eigen::Matrix3d::Identity() * dt;
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) = Eigen::Matrix3d::Identity() * sqrt(dt);
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) = Eigen::Matrix3d::Identity() * sqrt(dt);
  // perform Kalman prediction
  X_ = F_ * X_;
  P_ = F_ * P_ * F_.transpose() + B_ * Q_ * B_.transpose();
  return true;
}

bool Eskf::observe_pose(const Eigen::Matrix4d & pose, const Eigen::Matrix<double, 6, 1> & noise)
{
  // create measurement Y
  constexpr int kDimMeasurement = 6;
  Eigen::Matrix<double, kDimMeasurement, 1> Y;
  Y.block<3, 1>(0, 0) = pos_ - pose.block<3, 1>(0, 3);
  Eigen::Matrix3d dR = pose.block<3, 3>(0, 0).transpose() * ori_;
  Y.block<3, 1>(3, 0) = Sophus::SO3d::vee(dR - Eigen::Matrix3d::Identity());
  // measurement equation H, V
  Eigen::Matrix<double, kDimMeasurement, kDimState> H =
    Eigen::Matrix<double, kDimMeasurement, kDimState>::Zero();
  H.block<3, 3>(0, kIndexErrorPos) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(3, kIndexErrorOri) = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, kDimMeasurement, kDimMeasurement> V = noise.asDiagonal();
  // get kalman gain
  Eigen::Matrix<double, kDimState, kDimMeasurement> K;
  K = P_ * H.transpose() * (H * P_ * H.transpose() + V).inverse();
  // perform Kalman correct
  X_ = X_ + K * (Y - H * X_);
  P_ = (Eigen::Matrix<double, kDimState, kDimState>::Identity() - K * H) * P_;
  // eliminate error
  eliminate_error();
  // reset error state
  X_ = Eigen::Matrix<double, kDimState, 1>::Zero();
  return true;
}

double Eskf::get_time() {return time_;}

slt_common::ImuNavState Eskf::get_imu_nav_state()
{
  slt_common::ImuNavState nav_state;
  nav_state.time = time_;
  nav_state.position = pos_;
  nav_state.orientation = ori_;
  nav_state.linear_velocity = vel_;
  nav_state.gravity = gravity_;
  nav_state.gyro_bias = gyro_bias_;
  nav_state.accel_bias = accl_bias_;
  return nav_state;
}

void Eskf::eliminate_error(void)
{
  // update pos, vel, ori
  pos_ -= X_.block<3, 1>(kIndexErrorPos, 0);
  vel_ -= X_.block<3, 1>(kIndexErrorVel, 0);
  Eigen::Matrix3d dR =
    Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(X_.block<3, 1>(kIndexErrorOri, 0));
  Eigen::Matrix3d R = ori_ * dR;
  ori_ = Eigen::Quaterniond(R).normalized().toRotationMatrix();
  // update bias
  if (is_cov_stable(kIndexErrorGyro)) {
    gyro_bias_ -= X_.block<3, 1>(kIndexErrorGyro, 0);
  }
  if (is_cov_stable(kIndexErrorAccel)) {
    accl_bias_ -= X_.block<3, 1>(kIndexErrorAccel, 0);
  }
  // update imu integration
  slt_common::ImuNavState state;
  state.position = pos_;
  state.orientation = ori_;
  state.linear_velocity = vel_;
  state.accel_bias = accl_bias_;
  state.gyro_bias = gyro_bias_;
  state.gravity = gravity_;
  imu_integration_->set_state(state);
}

bool Eskf::is_cov_stable(int index_offset, double thresh)
{
  for (int i = 0; i < 3; ++i) {
    if (P_(index_offset + i, index_offset + i) > thresh) {
      return false;
    }
  }
  return true;
}

}  // namespace slt_eskf_locator
