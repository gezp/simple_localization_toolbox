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

#include "slt_common/subscriber/odometry_subscriber.hpp"

namespace slt_common
{

OdometrySubscriber::OdometrySubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    topic_name, buff_size,
    std::bind(&OdometrySubscriber::msg_callback, this, std::placeholders::_1));
}

void OdometrySubscriber::msg_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  buffer_mutex_.lock();
  msg_buffer_.push_back(msg);
  buffer_mutex_.unlock();
}

const std::string & OdometrySubscriber::get_frame_id() const
{
  return frame_id_;
}

const std::string & OdometrySubscriber::get_child_frame_id() const
{
  return child_frame_id_;
}

void OdometrySubscriber::parse_data(std::deque<OdomData> & output)
{
  std::vector<nav_msgs::msg::Odometry::SharedPtr> msg_buffer;
  buffer_mutex_.lock();
  msg_buffer.swap(msg_buffer_);
  buffer_mutex_.unlock();
  if (msg_buffer.empty()) {
    return;
  }
  if (frame_id_.empty()) {
    const auto & first_msg = msg_buffer.front();
    frame_id_ = first_msg->header.frame_id;
    child_frame_id_ = first_msg->child_frame_id;
  }
  for (auto & msg : msg_buffer) {
    OdomData odom_data;
    odom_data.time = rclcpp::Time(msg->header.stamp).seconds();

    // set the position:
    odom_data.pose(0, 3) = msg->pose.pose.position.x;
    odom_data.pose(1, 3) = msg->pose.pose.position.y;
    odom_data.pose(2, 3) = msg->pose.pose.position.z;

    // set the orientation:
    Eigen::Quaterniond q;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    odom_data.pose.block<3, 3>(0, 0) = q.matrix();

    // set the linear velocity:
    odom_data.linear_velocity.x() = msg->twist.twist.linear.x;
    odom_data.linear_velocity.y() = msg->twist.twist.linear.y;
    odom_data.linear_velocity.z() = msg->twist.twist.linear.z;

    // set the angular velocity:
    odom_data.angular_velocity.x() = msg->twist.twist.angular.x;
    odom_data.angular_velocity.y() = msg->twist.twist.angular.y;
    odom_data.angular_velocity.z() = msg->twist.twist.angular.z;

    output.push_back(odom_data);
  }
}

}  // namespace slt_common
