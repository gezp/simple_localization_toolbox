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

#include "slt_imu_odometry/imu_pre_integration.hpp"

#include <sophus/so3.hpp>

namespace slt_imu_odometry
{

ImuPreIntegration::ImuPreIntegration(
  double accel_noise, double gyro_noise, double accel_bias_noise, double gyro_bias_noise)
{
  // init bias
  ba_i_ = Eigen::Vector3d::Zero();
  bg_i_ = Eigen::Vector3d::Zero();
  // process noise
  const double accel_var = accel_noise * accel_noise;
  const double gyro_var = gyro_noise * gyro_noise;
  const double accel_bias_var = accel_bias_noise * accel_bias_noise;
  const double gyro_bias_var = gyro_bias_noise * gyro_bias_noise;
  Q_.setZero();
  Q_.block<3, 3>(INDEX_N_A_PREV, INDEX_N_A_PREV) = accel_var * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_N_A_CURR, INDEX_N_A_CURR) = accel_var * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_N_G_PREV, INDEX_N_G_PREV) = gyro_var * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_N_G_CURR, INDEX_N_G_CURR) = gyro_var * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_N_B_A, INDEX_N_B_A) = accel_bias_var * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(INDEX_N_B_G, INDEX_N_B_G) = gyro_bias_var * Eigen::Matrix3d::Identity();
  // process equation, state propagation
  F_.setZero();
  F_.block<3, 3>(INDEX_ALPHA, INDEX_BETA) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(INDEX_THETA, INDEX_B_G) = -Eigen::Matrix3d::Identity();

  // process equation, noise input
  B_.setZero();
  B_.block<3, 3>(INDEX_THETA, INDEX_N_G_PREV) = 0.50 * Eigen::Matrix3d::Identity();
  B_.block<3, 3>(INDEX_THETA, INDEX_N_G_CURR) = 0.50 * Eigen::Matrix3d::Identity();
  B_.block<3, 3>(INDEX_B_A, INDEX_N_B_A) = Eigen::Matrix3d::Identity();
  B_.block<3, 3>(INDEX_B_G, INDEX_N_B_G) = Eigen::Matrix3d::Identity();
  //
  reset();
}

void ImuPreIntegration::set_bias(Eigen::Vector3d ba, Eigen::Vector3d bg)
{
  ba_i_ = ba;
  bg_i_ = bg;
}

bool ImuPreIntegration::integrate(const slt_common::ImuData & imu_data)
{
  if (!is_inited_) {
    imu_data_buff_.push_back(imu_data);
    is_inited_ = true;
    return true;
  }

  if (imu_data.time <= imu_data_buff_.back().time) {
    return false;
  }

  // set buffer:
  imu_data_buff_.push_back(imu_data);

  // update state mean, covariance and Jacobian:
  update_state();
  return true;
}

void ImuPreIntegration::update_state(void)
{
  // the interval's two newest samples
  const slt_common::ImuData & prev = imu_data_buff_.at(imu_data_buff_.size() - 2);
  const slt_common::ImuData & curr = imu_data_buff_.at(imu_data_buff_.size() - 1);
  Eigen::Vector3d w0 = prev.angular_velocity - bg_i_;
  Eigen::Vector3d w1 = curr.angular_velocity - bg_i_;
  Eigen::Vector3d a0 = prev.linear_acceleration - ba_i_;
  Eigen::Vector3d a1 = curr.linear_acceleration - ba_i_;
  double dt = curr.time - prev.time;
  // update
  Eigen::Vector3d w_mid = 0.5 * (w0 + w1);
  Eigen::Matrix3d new_theta_ij = theta_ij_ * Sophus::SO3d::exp(w_mid * dt).matrix();
  Eigen::Vector3d a_mid = 0.5 * (theta_ij_ * a0 + new_theta_ij * a1);
  Eigen::Vector3d new_beta_ij = beta_ij_ + a_mid * dt;
  Eigen::Vector3d new_alpha_ij = alpha_ij_ + beta_ij_ * dt + 0.5 * a_mid * dt * dt;
  // update covariance
  // intermediate results
  auto dt2 = dt * dt;
  Eigen::Matrix3d prev_R = theta_ij_;
  Eigen::Matrix3d curr_R = new_theta_ij;
  Eigen::Matrix3d prev_R_a_hat = prev_R * Sophus::SO3d::hat(a0);
  Eigen::Matrix3d curr_R_a_hat = curr_R * Sophus::SO3d::hat(a1);
  Eigen::Matrix3d dR_inv = Eigen::Matrix3d::Identity() - Sophus::SO3d::hat(w_mid) * dt;
  // set up F: F = I + F_ * T
  // F12 & F14 & F15:
  F_.block<3, 3>(INDEX_ALPHA, INDEX_THETA) = -0.25 * (prev_R_a_hat + curr_R_a_hat * dR_inv) * dt;
  F_.block<3, 3>(INDEX_ALPHA, INDEX_B_A) = -0.25 * (prev_R + curr_R) * dt;
  F_.block<3, 3>(INDEX_ALPHA, INDEX_B_G) = 0.25 * curr_R_a_hat * dt2;
  // F22:
  F_.block<3, 3>(INDEX_THETA, INDEX_THETA) = -Sophus::SO3d::hat(w_mid);
  // F32 & F34 & F35
  F_.block<3, 3>(INDEX_BETA, INDEX_THETA) = -0.5 * (prev_R_a_hat + curr_R_a_hat * dR_inv);
  F_.block<3, 3>(INDEX_BETA, INDEX_B_A) = -0.5 * (prev_R + curr_R);
  F_.block<3, 3>(INDEX_BETA, INDEX_B_G) = 0.5 * curr_R_a_hat * dt;
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> F;
  F = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() + F_ * dt;
  // set up B: B + B_ * T
  // B11 & B12 & B13 & B14:
  B_.block<3, 3>(INDEX_ALPHA, INDEX_N_A_PREV) = 0.25 * prev_R * dt;
  B_.block<3, 3>(INDEX_ALPHA, INDEX_N_G_PREV) = -0.125 * curr_R_a_hat * dt2;
  B_.block<3, 3>(INDEX_ALPHA, INDEX_N_A_CURR) = 0.25 * curr_R * dt;
  B_.block<3, 3>(INDEX_ALPHA, INDEX_N_G_CURR) = -0.125 * curr_R_a_hat * dt2;
  // B31 & B32 & B33 & B34:
  B_.block<3, 3>(INDEX_BETA, INDEX_N_A_PREV) = 0.5 * prev_R;
  B_.block<3, 3>(INDEX_BETA, INDEX_N_G_PREV) = -0.25 * curr_R_a_hat * dt;
  B_.block<3, 3>(INDEX_BETA, INDEX_N_A_CURR) = 0.5 * curr_R;
  B_.block<3, 3>(INDEX_BETA, INDEX_N_G_CURR) = -0.25 * curr_R_a_hat * dt;
  Eigen::Matrix<double, DIM_STATE, DIM_NOISE> B = B_ * dt;
  // update P_:
  P_ = F * P_ * F.transpose() + B * Q_ * B.transpose();
  // update Jacobian:
  J_ = F * J_;
  // update
  alpha_ij_ = new_alpha_ij;
  theta_ij_ = new_theta_ij;
  beta_ij_ = new_beta_ij;
}

void ImuPreIntegration::reset()
{
  imu_data_buff_.clear();
  is_inited_ = false;
  alpha_ij_ = Eigen::Vector3d::Zero();
  theta_ij_ = Eigen::Matrix3d::Identity();
  beta_ij_ = Eigen::Vector3d::Zero();
  P_.setZero();
  J_.setIdentity();
}

void ImuPreIntegration::reintegrate()
{
  const std::deque<slt_common::ImuData> samples = imu_data_buff_;
  reset();
  for (const auto & imu_data : samples) {
    integrate(imu_data);
  }
}

double ImuPreIntegration::get_delta_time() const
{
  return imu_data_buff_.size() < 2 ? 0.0 : imu_data_buff_.back().time - imu_data_buff_.front().time;
}

Eigen::Vector3d ImuPreIntegration::get_alpha() {return alpha_ij_;}

Eigen::Matrix3d ImuPreIntegration::get_theta() {return theta_ij_;}

Eigen::Vector3d ImuPreIntegration::get_beta() {return beta_ij_;}

Eigen::Matrix<double, 15, 15> ImuPreIntegration::get_covariance() {return P_;}

Eigen::Matrix<double, 15, 15> ImuPreIntegration::get_jacobian() {return J_;}

Eigen::Vector3d ImuPreIntegration::get_ba() {return ba_i_;}

Eigen::Vector3d ImuPreIntegration::get_bg() {return bg_i_;}

slt_common::ImuData ImuPreIntegration::get_imu_data() const
{
  return imu_data_buff_.empty() ? slt_common::ImuData{} : imu_data_buff_.back();
}

const std::deque<slt_common::ImuData> & ImuPreIntegration::get_imu_data_buffer() const
{
  return imu_data_buff_;
}

bool ImuPreIntegration::apply(
  const slt_common::ImuNavState & from, slt_common::ImuNavState & to) const
{
  if (imu_data_buff_.empty()) {
    return false;
  }
  auto & p = from.position;
  auto & r = from.orientation;
  auto & v = from.linear_velocity;
  auto & g = from.gravity;
  to = from;
  to.time = imu_data_buff_.back().time;
  const double dt = get_delta_time();
  to.position = p + v * dt + 0.5 * g * dt * dt + r * alpha_ij_;
  to.linear_velocity = v + g * dt + r * beta_ij_;
  to.orientation = Eigen::Quaterniond{r * theta_ij_}.normalized().toRotationMatrix();
  return true;
}
}  // namespace slt_imu_odometry
