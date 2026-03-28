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

#include "slt_common/sensor_data/imu_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace slt_common
{
class ImuSubscriber
{
public:
  ImuSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size);
  ImuSubscriber() = default;
  void parse_data(std::deque<ImuData> & output);
  void parse_data(std::deque<ImuData2> & output);

private:
  void msg_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;

  std::deque<ImuData2> data_buffer_;
  std::mutex buffer_mutex_;
};
}  // namespace slt_common
