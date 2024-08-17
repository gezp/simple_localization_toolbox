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

#include <memory>
#include <deque>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "localization_common/publisher/cloud_publisher.hpp"
#include "localization_common/publisher/odometry_publisher.hpp"
#include "localization_common/subscriber/cloud_subscriber.hpp"
#include "localization_common/subscriber/odometry_subscriber.hpp"
#include "localization_common/extrinsics_manager.hpp"
#include "localization_common/msg_utils.hpp"
#include "localization_common/sensor_data_utils.hpp"
#include "localization_common/odom_data_buffer.hpp"
#include "localization_common/tic_toc.hpp"
#include "lidar_odometry/simple_odometry.hpp"
#include "lidar_odometry/loam_odometry.hpp"

namespace lidar_odometry
{

class LidarOdometryNode
{
  using LidarMsgData = localization_common::CloudSubscriber::MsgData;

public:
  enum OdometryMethod { Simple, Loam, Unknown };
  explicit LidarOdometryNode(rclcpp::Node::SharedPtr node);
  ~LidarOdometryNode();

private:
  bool run();
  bool get_initial_pose_by_reference_odom(
    double time, Eigen::Matrix4d & initial_pose, bool & is_old_data);
  void set_extrinsics_for_odometry(OdometryMethod method, const Eigen::Matrix4d & T_base_lidar);
  bool update_odometry(OdometryMethod method, const LidarMsgData & msg_data);
  localization_common::OdomData align_odom_to_map(const localization_common::OdomData & odom);
  void publish_odom(const localization_common::OdomData & odom);
  void publish_data(OdometryMethod method);

private:
  rclcpp::Node::SharedPtr node_;
  // pub & sub
  std::shared_ptr<localization_common::CloudSubscriber> cloud_sub_;
  std::shared_ptr<localization_common::OdometrySubscriber> reference_odom_sub_;
  std::shared_ptr<localization_common::CloudPublisher> current_scan_pub_;
  std::shared_ptr<localization_common::CloudPublisher> local_map_pub_;
  std::shared_ptr<localization_common::CloudPublisher> loam_feature_pub_;
  std::shared_ptr<localization_common::OdometryPublisher> lidar_odom_pub_;
  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<localization_common::ExtrinsicsManager> extrinsics_manager_;
  std::string lidar_frame_id_{"lidar"};
  std::string base_frame_id_{"base"};
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  bool is_valid_extrinsics_{false};
  bool publish_tf_{false};
  // front end tool and thread
  OdometryMethod odometry_method_{OdometryMethod::Unknown};
  std::shared_ptr<SimpleOdometry> simple_odometry_;
  std::shared_ptr<LoamOdometry> loam_odometry_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::deque<LidarMsgData> lidar_data_buffer_;
  std::shared_ptr<localization_common::OdomDataBuffer> ref_odom_buffer_;
  localization_common::TwistData last_twist_;
  // params
  Eigen::Matrix4d T_map_odom_ = Eigen::Matrix4d::Identity();
  bool use_initial_pose_from_topic_{false};
  bool undistort_point_cloud_{false};
  bool inited_{false};
  // debug
  localization_common::AdvancedTicToc elapsed_time_statistics_;
};

}  // namespace lidar_odometry
