// Copyright 2026 Gezp (https://github.com/gezp).
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

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "slt_common/camera_model/camera_intrinsic_param.hpp"

namespace slt_common
{
// the publisher latches the intrinsics, so transient_local durability is requested: the
// subscriber still gets them when it starts after the publisher
class CameraInfoSubscriber
{
public:
  CameraInfoSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name);
  CameraInfoSubscriber() = default;
  // returns false until the first camera info arrives
  bool get_param(CameraIntrinsicParam & output);

private:
  void msg_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_ptr);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_;

  CameraIntrinsicParam param_;
  bool has_param_{false};
  std::mutex param_mutex_;
};
}  // namespace slt_common
