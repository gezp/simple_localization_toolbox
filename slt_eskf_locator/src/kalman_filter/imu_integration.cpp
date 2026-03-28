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

#include "slt_eskf_locator/kalman_filter/imu_integration.hpp"

#include <sophus/so3.hpp>

namespace slt_eskf_locator
{

ImuIntegration::ImuIntegration() {}

bool ImuIntegration::init(
  const slt_common::ImuNavState & state, const slt_common::ImuData & imu_data)
{
  state_ = state;
  imu_data_buff_.clear();
  imu_data_buff_.push_back(imu_data);
  imu_data_buff_.push_back(imu_data);
  return true;
}

bool ImuIntegration::integrate(const slt_common::ImuData & imu_data)
{
  // check
  if (imu_data.time < state_.time) {
    return false;
  }
  imu_data_buff_.push_back(imu_data);
  imu_data_buff_.pop_front();
  // get deltas:
  double dt = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;
  // phi
  Eigen::Vector3d w0 = imu_data_buff_.at(0).angular_velocity - state_.gyro_bias;
  Eigen::Vector3d w1 = imu_data_buff_.at(1).angular_velocity - state_.gyro_bias;
  Eigen::Vector3d phi = 0.5 * (w0 + w1) * dt;
  // ori
  Eigen::Matrix3d new_ori = state_.orientation * Sophus::SO3d::exp(phi).matrix();
  // vel
  Eigen::Vector3d a0 = imu_data_buff_.at(0).linear_acceleration - state_.accel_bias;
  Eigen::Vector3d a1 = imu_data_buff_.at(1).linear_acceleration - state_.accel_bias;
  Eigen::Vector3d a = 0.5 * (state_.orientation * a0 + new_ori * a1);
  Eigen::Vector3d new_vel = state_.linear_velocity + (a + state_.gravity) * dt;
  // pos
  Eigen::Vector3d new_pos = state_.position + 0.5 * (state_.linear_velocity + new_vel) * dt;
  // update
  state_.time = imu_data.time;
  state_.position = new_pos;
  state_.orientation = new_ori;
  state_.linear_velocity = new_vel;
  return true;
}

void ImuIntegration::set_state(const slt_common::ImuNavState & state) {state_ = state;}

const slt_common::ImuNavState & ImuIntegration::get_state() {return state_;}

}  // namespace slt_eskf_locator
