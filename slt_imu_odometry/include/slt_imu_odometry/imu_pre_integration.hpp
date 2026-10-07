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

#pragma once

#include <Eigen/Dense>
#include <deque>
#include <memory>

#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"

namespace slt_imu_odometry
{

class ImuPreIntegration
{
  static const int DIM_STATE = 15;
  static const int DIM_NOISE = 18;

  static const int INDEX_ALPHA = 0;
  static const int INDEX_THETA = 3;
  static const int INDEX_BETA = 6;
  static const int INDEX_B_A = 9;
  static const int INDEX_B_G = 12;

  static const int INDEX_N_A_PREV = 0;
  static const int INDEX_N_G_PREV = 3;
  static const int INDEX_N_A_CURR = 6;
  static const int INDEX_N_G_CURR = 9;
  static const int INDEX_N_B_A = 12;
  static const int INDEX_N_B_G = 15;

public:
  ImuPreIntegration(
    double accel_noise, double gyro_noise, double accel_bias_noise, double gyro_bias_noise);
  void set_bias(Eigen::Vector3d ba, Eigen::Vector3d bg);
  bool integrate(const slt_common::ImuData & imu_data);
  void reintegrate();
  void reset();
  // the interval spans its buffer, first sample to last: zero while it holds fewer
  double get_delta_time() const;
  Eigen::Vector3d get_alpha();
  Eigen::Matrix3d get_theta();
  Eigen::Vector3d get_beta();
  Eigen::Matrix<double, 15, 15> get_covariance();
  Eigen::Matrix<double, 15, 15> get_jacobian();
  Eigen::Vector3d get_ba();
  Eigen::Vector3d get_bg();
  // the last integrated sample, the interval's end: the next interval opens there. a default
  // sample while the interval holds none
  slt_common::ImuData get_imu_data() const;
  const std::deque<slt_common::ImuData> & get_imu_data_buffer() const;
  bool apply(
    const slt_common::ImuNavState & from, slt_common::ImuNavState & to) const;

private:
  void update_state();

private:
  // hyper-params:
  double prior_noise_;
  double gyro_noise_;
  double accel_noise_;
  double gyro_bias_noise_;
  double accel_bias_noise_;
  // data buff:
  std::deque<slt_common::ImuData> imu_data_buff_;
  // process noise:
  Eigen::Matrix<double, DIM_NOISE, DIM_NOISE> Q_;
  // process equation:
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> F_;
  Eigen::Matrix<double, DIM_STATE, DIM_NOISE> B_;
  // pre-integration state:
  // relative translation
  Eigen::Vector3d alpha_ij_;
  // relative orientation
  Eigen::Matrix3d theta_ij_;
  // relative velocity
  Eigen::Vector3d beta_ij_;
  // accel bias
  Eigen::Vector3d ba_i_;
  // gyro bias
  Eigen::Vector3d bg_i_;
  // covariance matrix
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> P_;
  // Jacobian for update caused by bias
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> J_;
  bool is_inited_{false};
};

using ImuPreIntegrationPtr = std::shared_ptr<ImuPreIntegration>;

}  // namespace slt_imu_odometry
