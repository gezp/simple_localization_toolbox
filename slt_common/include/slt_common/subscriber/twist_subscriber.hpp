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

#include <deque>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "slt_common/sensor_data/twist_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace slt_common
{
class TwistSubscriber
{
public:
  TwistSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size);
  TwistSubscriber() = default;
  void parse_data(std::deque<TwistData> & deque_twist_data);

private:
  void msg_callback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_;

  std::deque<TwistData> new_twist_data_;
  std::mutex buff_mutex_;
};
}  // namespace slt_common
