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

#include <deque>
#include <memory>

#include "slt_common/point_cloud_registration/point_cloud_registration_interface.hpp"
#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/sensor_data/twist_data.hpp"

namespace slt_lio
{

// odometry state initialization: builds the initial navigation state from the buffered
// imu/lidar data (the covariance is Eskf::init_state's business), returned with the imu
// sample it is defined at, which seeds the filter integration.
// the method config picks the initial velocity:
//   static: at rest, zero
//   dynamic: scan matching of the newest scans
//   auto: dynamic first, static as a fallback when scan matching fails
class LioInitializer
{
public:
  explicit LioInitializer(const YAML::Node & config);
  // T_imu_lidar: lidar -> imu, the scan matching measures the lidar origin's velocity
  void set_extrinsic(const Eigen::Matrix4d & T_imu_lidar);
  // the caller forwards data until the initialization is done
  bool add_imu_data(const slt_common::ImuData & imu_data);
  bool add_lidar_data(const slt_common::LidarData & lidar_data);
  // true when the initialization is done (from then on a no-op returning true)
  bool try_init(slt_common::ImuNavState & state, slt_common::ImuData & imu_data);
  // drop the buffered data, the initializer is ready for a new initialization
  void reset();

private:
  // the imu window is at rest: small gyro & accel variance, accel near gravity
  bool is_static(Eigen::Vector3d & gyro_avg, Eigen::Vector3d & accl_avg) const;
  // state at rest, velocity zero
  bool init_static(slt_common::ImuNavState & state, slt_common::ImuData & imu_data);
  // state from the scan matching velocity (the imu does not observe the velocity)
  bool init_dynamic(slt_common::ImuNavState & state, slt_common::ImuData & imu_data);
  // twist of the lidar at the newest scan, by the scan matching of the newest scan pair
  bool init_twist_by_lidar(slt_common::TwistData & twist);
  // imu sample nearest to `time`, false when none is within kMaxImuTimeDiff
  bool find_nearest_imu(double time, slt_common::ImuData & imu_data) const;

private:
  std::string method_{"auto"};
  // static detection: samples averaged & thresholds, and the pre-init imu window size
  int static_init_num_{20};
  double static_gyro_threshold_{0.05};
  double static_accel_threshold_{1.0};
  double static_accel_noise_{0.1};
  // earth gravity magnitude: the accel at rest measures the specific force
  double gravity_magnitude_{9.80943};
  // velocity by scan matching of the newest scan pair
  static constexpr size_t kNumInitScan = 2;
  // imu window bound: the dynamic init seed is near the newest scan
  static constexpr size_t kMaxImuBuffer = 500;
  // a seed is only valid when the sample is that close to the scan end time
  static constexpr double kMaxImuTimeDiff = 0.05;
  std::shared_ptr<slt_common::PointCloudRegistrationInterface> registration_;
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  // data window of the initializer, cleared by reset()
  std::deque<slt_common::ImuData> imu_buffer_;
  std::deque<slt_common::LidarData> lidar_buffer_;
  // time of the last scan used by a scan matching attempt
  double last_registration_time_{0.0};
  bool is_inited_{false};
};

}  // namespace slt_lio
