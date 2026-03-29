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

#include <thread>
#include <memory>
#include <deque>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
// subscriber
#include "slt_common/subscriber/cloud_subscriber.hpp"
#include "slt_common/subscriber/nav_sat_fix_subscriber.hpp"
#include "slt_common/subscriber/imu_subscriber.hpp"
#include "slt_common/subscriber/twist_subscriber.hpp"
// publisher
#include "slt_common/publisher/cloud_publisher.hpp"
#include "slt_common/publisher/gnss_publisher.hpp"
#include "slt_common/publisher/odometry_publisher.hpp"
//
#include "slt_common/extrinsics_manager.hpp"
#include "slt_common/odom_data_buffer.hpp"

namespace slt_common
{
class KittiPreprocessNode
{
public:
  explicit KittiPreprocessNode(rclcpp::Node::SharedPtr node);

private:
  bool run();
  bool read_data();
  bool has_gnss_data();
  bool valid_gnss_data();

private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_;
  std::shared_ptr<NavSatFixSubscriber> nav_sat_fix_sub_;
  std::shared_ptr<TwistSubscriber> twist_sub_;
  std::shared_ptr<ImuSubscriber> imu_sub_;
  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_;
  std::shared_ptr<GnssPublisher> gnss_data_pub_;
  std::shared_ptr<OdometryPublisher> gnss_odom_pub_;
  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<ExtrinsicsManager> extrinsics_manager_;
  std::string imu_frame_id_{"imu"};
  std::string lidar_frame_id_{"lidar"};
  std::string base_frame_id_{"base"};
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  bool is_valid_extrinsics_{false};
  bool publish_tf_{false};
  // thread
  std::unique_ptr<std::thread> run_thread_;
  bool run_flag_{true};
  // map origin (latitude, longitude, altitude)
  bool use_manual_map_origin_{true};
  std::vector<double> map_origin_{48.982545, 8.390366, 116.382141};
  // data
  std::deque<LidarData> lidar_data_buffer_;
  std::deque<ImuData> imu_data_buffer_;
  std::deque<TwistData> twist_data_buffer_;
  std::deque<GnssData> gnss_data_buffer_;

  ImuData current_imu_data_;
  TwistData current_twist_data_;
  GnssData current_gnss_data_;
  std::shared_ptr<OdomDataBuffer> gnss_odom_buffer_;
};
}  // namespace slt_common
