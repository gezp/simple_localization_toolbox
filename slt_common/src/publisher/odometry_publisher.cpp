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

#include "slt_common/publisher/odometry_publisher.hpp"

namespace slt_common
{
OdometryPublisher::OdometryPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string base_frame_id,
  std::string child_frame_id, int buff_size)
: node_(node)
{
  publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name, buff_size);
  odometry_.header.frame_id = base_frame_id;
  odometry_.child_frame_id = child_frame_id;
}

void OdometryPublisher::publish(const OdomData & odom)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(odom.time * 1e9));
  odometry_.header.stamp = ros_time;

  // set the position
  odometry_.pose.pose.position.x = odom.pose(0, 3);
  odometry_.pose.pose.position.y = odom.pose(1, 3);
  odometry_.pose.pose.position.z = odom.pose(2, 3);

  Eigen::Quaterniond q;
  q = odom.pose.block<3, 3>(0, 0);
  q.normalize();
  odometry_.pose.pose.orientation.x = q.x();
  odometry_.pose.pose.orientation.y = q.y();
  odometry_.pose.pose.orientation.z = q.z();
  odometry_.pose.pose.orientation.w = q.w();

  // set the twist:
  odometry_.twist.twist.linear.x = odom.linear_velocity.x();
  odometry_.twist.twist.linear.y = odom.linear_velocity.y();
  odometry_.twist.twist.linear.z = odom.linear_velocity.z();
  odometry_.twist.twist.angular.x = odom.angular_velocity.x();
  odometry_.twist.twist.angular.y = odom.angular_velocity.y();
  odometry_.twist.twist.angular.z = odom.angular_velocity.z();

  publisher_->publish(odometry_);
}

void OdometryPublisher::publish(const Eigen::Matrix4d & pose, double time)
{
  OdomData odom;
  odom.time = time;
  odom.pose = pose;
  publish(odom);
}

bool OdometryPublisher::has_subscribers() {return publisher_->get_subscription_count() > 0;}

}  // namespace slt_common
