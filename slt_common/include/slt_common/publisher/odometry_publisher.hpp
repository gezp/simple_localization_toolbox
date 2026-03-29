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

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "slt_common/sensor_data/odom_data.hpp"

namespace slt_common
{
class OdometryPublisher
{
public:
  OdometryPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string base_frame_id,
    std::string child_frame_id, int buff_size);
  void set_tf_broadcaster(std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub);
  void publish(const OdomData & odom);
  void publish(const Eigen::Matrix4d & pose, double time);
  bool has_subscribers();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  nav_msgs::msg::Odometry odometry_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};
}  // namespace slt_common
