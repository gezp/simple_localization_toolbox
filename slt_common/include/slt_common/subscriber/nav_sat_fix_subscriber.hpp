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
#include <GeographicLib/LocalCartesian.hpp>

#include "slt_common/sensor_data/gnss_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace slt_common
{
class NavSatFixSubscriber
{
public:
  NavSatFixSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size);
  void set_map_origin(double latitude, double longitude, double altitude);
  void parse_data(std::deque<GnssData> & output);

private:
  void msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_;
  std::deque<GnssData> buffer_;
  std::mutex buffer_mutex_;
  // geo_converter
  GeographicLib::LocalCartesian geo_converter_;
  bool origin_position_inited_{false};
};
}  // namespace slt_common
