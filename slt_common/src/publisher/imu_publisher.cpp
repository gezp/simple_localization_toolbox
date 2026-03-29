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

#include "slt_common/publisher/imu_publisher.hpp"

namespace slt_common
{

ImuPublisher::ImuPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buff_size)
: node_(node), frame_id_(frame_id)
{
  publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_name, buff_size);

  imu_.header.frame_id = frame_id_;
}

void ImuPublisher::publish(const ImuData & imu_data)
{
  rclcpp::Time ros_time(static_cast<uint64_t>(imu_data.time * 1e9));
  imu_.header.stamp = ros_time;

  // set angular velocity:
  imu_.angular_velocity.x = imu_data.angular_velocity.x();
  imu_.angular_velocity.y = imu_data.angular_velocity.y();
  imu_.angular_velocity.z = imu_data.angular_velocity.z();

  // set linear acceleration:
  imu_.linear_acceleration.x = imu_data.linear_acceleration.x();
  imu_.linear_acceleration.y = imu_data.linear_acceleration.y();
  imu_.linear_acceleration.z = imu_data.linear_acceleration.z();

  // set orientation:
  if (imu_data.has_orientation) {
    imu_.orientation.w = imu_data.orientation.w();
    imu_.orientation.x = imu_data.orientation.x();
    imu_.orientation.y = imu_data.orientation.y();
    imu_.orientation.z = imu_data.orientation.z();
  } else {
    imu_.orientation.w = 0.0;
    imu_.orientation.x = 0.0;
    imu_.orientation.y = 0.0;
    imu_.orientation.z = 0.0;
    imu_.orientation_covariance[0] = -1.0;
  }

  publisher_->publish(imu_);
}

bool ImuPublisher::has_subscribers(void)
{
  return publisher_->get_subscription_count() > 0;
}

}  // namespace slt_common
