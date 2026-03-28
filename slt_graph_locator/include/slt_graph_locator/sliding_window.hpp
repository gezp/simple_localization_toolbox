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

#include <deque>
#include <string>
#include <memory>
#include <vector>

#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/odom_data.hpp"
#include "slt_common/odom_data_buffer.hpp"
#include "slt_imu_odometry/imu_integration.hpp"
#include "slt_graph_locator/graph_optimizer/ceres_graph_optimizer.hpp"

namespace slt_graph_locator
{

class SlidingWindow
{
  struct KeyFrame
  {
    slt_common::OdomData lidar_pose;
    slt_common::OdomData gnss_pose;
    std::vector<slt_common::ImuData> pre_integration_buffer;
    bool has_valid_gnss;
    // just to estimate velocity for some cases (not used now)
    slt_common::OdomData last_lidar_pose;
  };

public:
  SlidingWindow();
  ~SlidingWindow();
  bool init_with_config(const YAML::Node & config_node);
  void set_extrinsic(const Eigen::Matrix4d & T_base_imu);
  bool add_imu_data(const slt_common::ImuData & imu);
  bool add_lidar_pose(const slt_common::OdomData & lidar_pose);
  bool add_gnss_pose(const slt_common::OdomData & gnss_pose);
  bool update();
  slt_common::ImuNavState get_imu_nav_state();
  slt_common::OdomData get_current_odom();

private:
  bool init_graph_optimizer(const YAML::Node & config_node);
  bool check_valid_lidar();
  bool check_new_key_frame(const slt_common::OdomData & odom);
  int create_graph_node(double time, const Eigen::Matrix4d & pose, const Eigen::Vector3d & vel);
  int create_graph_node_from_odom(const slt_common::OdomData & odom);
  int create_graph_node_from_lidar(
    const slt_common::OdomData & pose,
    const slt_common::OdomData & neighbor_pose);
  int create_graph_node_from_imu(const std::vector<slt_common::ImuData> & imus);
  bool get_synced_imu_buffer(double time, std::vector<slt_common::ImuData> & buffer);
  bool try_init_from_lidar();
  bool update_graph(const KeyFrame & key_frame);

private:
  // optimizer:
  std::shared_ptr<CeresGraphOptimizer> graph_optimizer_;
  std::shared_ptr<slt_imu_odometry::ImuIntegration> imu_integration_;
  // params
  struct
  {
    float max_distance;
    float max_interval;
  } key_frame_config_;
  bool use_gnss_ = true;
  bool use_lidar_pose_ = false;
  bool use_imu_pre_integration_ = false;
  double gravity_magnitude_;
  size_t sliding_window_size_;
  // measurement noise
  Eigen::VectorXd lidar_odometry_noise_;
  Eigen::VectorXd lidar_pose_noise_;
  Eigen::VectorXd gnss_position_noise_;
  // imu noise
  ImuConfig imu_config_;
  //
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  // sensor data
  std::deque<slt_common::ImuData> imu_buffer_;
  std::deque<slt_common::OdomData> lidar_pose_buffer_;
  std::shared_ptr<slt_common::OdomDataBuffer> gnss_odom_buffer_;
  size_t unhandled_lidar_idx_{0};
  size_t unhandled_imu_idx_{0};
  // odom result
  slt_common::ImuNavState current_imu_nav_state_;
  slt_common::ImuData current_imu_;
  // key frame
  slt_common::OdomData latest_key_pose_;
  slt_common::ImuData lastest_key_imu_;
  //
  bool has_inited_{false};
};

}  // namespace slt_graph_locator
