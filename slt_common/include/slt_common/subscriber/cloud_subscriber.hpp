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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"

namespace slt_common
{

class CloudSubscriber
{
public:
  CloudSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size);

  void parse_data(std::deque<LidarData> & output);

private:
  static bool has_field(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg, const std::string & field_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  std::deque<LidarData> buffer_;
  std::mutex buffer_mutex_;
};
}  // namespace slt_common
