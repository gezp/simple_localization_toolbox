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

#include "slt_common/subscriber/twist_subscriber.hpp"

namespace slt_common
{
TwistSubscriber::TwistSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
    topic_name, buff_size,
    std::bind(&TwistSubscriber::msg_callback, this, std::placeholders::_1));
}

void TwistSubscriber::msg_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg_ptr)
{
  TwistData twist_data;
  twist_data.time = rclcpp::Time(twist_msg_ptr->header.stamp).seconds();

  auto & v = twist_msg_ptr->twist.linear;
  twist_data.linear_velocity = Eigen::Vector3d(v.x, v.y, v.z);
  auto & w = twist_msg_ptr->twist.angular;
  twist_data.angular_velocity = Eigen::Vector3d(w.x, w.y, w.z);
  buff_mutex_.lock();
  new_twist_data_.push_back(twist_data);
  buff_mutex_.unlock();
}

void TwistSubscriber::parse_data(std::deque<TwistData> & twist_data_buff)
{
  buff_mutex_.lock();
  if (new_twist_data_.size() > 0) {
    twist_data_buff.insert(
      twist_data_buff.end(), new_twist_data_.begin(), new_twist_data_.end());
    new_twist_data_.clear();
  }
  buff_mutex_.unlock();
}
}  // namespace slt_common
