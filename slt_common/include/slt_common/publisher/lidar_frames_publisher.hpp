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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "slt_interface/msg/lidar_frames.hpp"
#include "slt_common/sensor_data/lidar_frame.hpp"

namespace slt_common
{
class LidarFramesPublisher
{
public:
  LidarFramesPublisher(rclcpp::Node::SharedPtr node, std::string topic_name, int buff_size);

  void publish(const std::vector<LidarFrame> & frames);
  bool has_subscribers() {return publisher_->get_subscription_count() > 0;}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<slt_interface::msg::LidarFrames>::SharedPtr publisher_;
};
}  // namespace slt_common
