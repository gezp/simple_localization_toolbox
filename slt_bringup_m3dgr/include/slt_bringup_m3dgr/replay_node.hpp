// Copyright 2026 Gezp (https://github.com/gezp).
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

#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "slt_common/publisher/odometry_publisher.hpp"
#include "slt_common/sensor_data/odom_data.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace slt_bringup_m3dgr
{

struct StaticTransform
{
  std::string frame_id;
  std::string child_frame_id;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
};

class ReplayNode
{
public:
  explicit ReplayNode(rclcpp::Node::SharedPtr node);
  ~ReplayNode();

private:
  bool load_extrinsics(const std::string & config_path);
  void publish_extrinsics();
  bool load_ground_truth(const std::string & gt_path);
  void rebuild_gt_rotation();
  void replay_ground_truth();

  rclcpp::Node::SharedPtr node_;
  std::vector<StaticTransform> extrinsics_;
  std::vector<slt_common::OdomData> ground_truth_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  std::shared_ptr<slt_common::OdometryPublisher> gt_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  double rate_ = 1.0;
  bool publish_tf_ = false;
  bool rebuild_gt_rotation_ = false;
  std::thread run_thread_;
  std::atomic<bool> exit_{false};
};

}  // namespace slt_bringup_m3dgr
