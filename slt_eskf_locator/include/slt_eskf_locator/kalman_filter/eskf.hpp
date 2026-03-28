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

#include <yaml-cpp/yaml.h>
#include <memory>

#include "slt_eskf_locator/kalman_filter/kalman_filter_interface.hpp"
#include "slt_eskf_locator/kalman_filter/imu_integration.hpp"

namespace slt_eskf_locator
{

class Eskf : public KalmanFilterInterface
{
public:
  explicit Eskf(const YAML::Node & node);
  ~Eskf() {}
  void init_state(
    const slt_common::ImuNavState & state,
    const slt_common::ImuData & imu_data) override;
  bool predict(const slt_common::ImuData & imu_data) override;
  bool observe_pose(
    const Eigen::Matrix4d & pose, const Eigen::Matrix<double, 6, 1> & noise) override;
  double get_time() override;
  slt_common::ImuNavState get_imu_nav_state() override;
  void print_info() override;

private:
  void eliminate_error(void);
  bool is_cov_stable(int index_offset, double thresh = 1.0e-5);

private:
  // state dim/index
  static constexpr int kDimState{15};
  static constexpr int kIndexErrorPos{0};
  static constexpr int kIndexErrorVel{3};
  static constexpr int kIndexErrorOri{6};
  static constexpr int kIndexErrorAccel{9};
  static constexpr int kIndexErrorGyro{12};
  // process nosie dim/index
  static constexpr int kDimProcessNoise{12};
  static constexpr int kIndexNoiseAccel{0};
  static constexpr int kIndexNoiseGyro{3};
  static constexpr int kIndexNoiseBiasAccel{6};
  static constexpr int kIndexNoiseBiasGyro{9};
  // nominal state
  Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ori_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accl_bias_ = Eigen::Vector3d::Zero();
  // state
  Eigen::Matrix<double, kDimState, 1> X_;
  Eigen::Matrix<double, kDimState, kDimState> P_;
  // process equations
  Eigen::Matrix<double, kDimState, kDimProcessNoise> B_;
  Eigen::Matrix<double, kDimState, kDimState> F_;
  Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise> Q_;
  // time
  double time_;
  // imu_integration
  std::shared_ptr<ImuIntegration> imu_integration_;
  Eigen::Vector3d gravity_;
  // covariance params:
  double prior_noise_;
  double gyro_noise_;
  double accel_noise_;
  double gyro_bias_noise_;
  double accel_bias_noise_;
};

}  // namespace slt_eskf_locator
