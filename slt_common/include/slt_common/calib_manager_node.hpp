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

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace slt_common
{
// publishes a rig calibration: every transform as a static tf, the first camera's intrinsics as a
// latched camera_info. the imu entries are ignored
class CalibManagerNode
{
public:
  explicit CalibManagerNode(rclcpp::Node::SharedPtr node);

private:
  struct StaticTransform
  {
    std::string frame_id;
    std::string child_frame_id;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  };

  bool load_calibration(const std::string & config_path);
  void publish_extrinsics();
  void publish_camera_info();

  rclcpp::Node::SharedPtr node_;
  std::vector<StaticTransform> extrinsics_;
  sensor_msgs::msg::CameraInfo camera_info_;
  bool has_camera_info_{false};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};
}  // namespace slt_common
