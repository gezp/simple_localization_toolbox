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

#pragma once

#include <yaml-cpp/yaml.h>
#include <sophus/so3.hpp>

#include <Eigen/Dense>
#include <deque>
#include <vector>

#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_imu_odometry/imu_integration.hpp"

namespace slt_lio
{

// Error-State Kalman Filter for tightly coupled lidar-inertial odometry,
// reference: FAST-LIO. error state (18): pos, vel, ori (body right perturbation),
// accel_bias, gyro_bias, gravity (world frame, estimated online). the observation is
// a batch of point-to-plane residuals, re-linearized at the updated state each
// iteration until convergence.
class Eskf
{
public:
  // error state index
  static constexpr int kDimState = 18;
  static constexpr int kDimProcessNoise = 12;
  enum StateIndex {
    kIndexErrorPos = 0,
    kIndexErrorVel = 3,
    kIndexErrorOri = 6,
    kIndexErrorAccel = 9,
    kIndexErrorGyro = 12,
    kIndexErrorGravity = 15,
  };
  enum NoiseIndex {
    kIndexNoiseAccel = 0,
    kIndexNoiseGyro = 3,
    kIndexNoiseBiasAccel = 6,
    kIndexNoiseBiasGyro = 9,
  };
  // point-to-plane observation: r = n' * (R * p_body + t - q_map)
  struct PlaneObservation
  {
    Eigen::Vector3d point;
    Eigen::Vector3d plane_point;
    Eigen::Vector3d plane_normal;
  };

  // config: the eskf section of the lio config (covariance, observation)
  explicit Eskf(const YAML::Node & config);
  ~Eskf() {}

  // set the nominal state & the prior covariance, the state comes from LioInitializer
  void init_state(const slt_common::ImuNavState & state, const slt_common::ImuData & imu_data);
  bool predict(const slt_common::ImuData & imu_data);
  bool observe_point_cloud(const std::vector<PlaneObservation> & observations);
  double get_time();
  slt_common::ImuNavState get_imu_nav_state();
  slt_common::ImuData get_imu_data();
  bool is_inited() { return is_inited_; }

private:
  void eliminate_error();

private:
  // imu forward integration (midpoint, bias-corrected)
  slt_imu_odometry::ImuIntegration imu_integration_;
  Eigen::Matrix<double, kDimState, kDimState> P_;
  Eigen::Matrix<double, kDimState, kDimState> F_;
  Eigen::Matrix<double, kDimState, kDimProcessNoise> B_;
  Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise> Q_;
  Eigen::Matrix<double, kDimState, 1> X_;
  // config values: prior covariance, process noise (variances) & observation
  double noise_prior_{1e-6};
  // gravity prior (variance): tight on purpose, a loose gravity wanders
  double noise_prior_gravity_{1e-4};
  double noise_gyro_{1e-4};
  double noise_accel_{2.5e-3};
  double noise_gyro_bias_{1e-5};
  double noise_accel_bias_{2.5e-4};
  int max_num_iterations_{4};
  double converge_threshold_{0.1};
  double observation_noise_{0.1};
  bool is_inited_{false};
};

}  // namespace slt_lio
