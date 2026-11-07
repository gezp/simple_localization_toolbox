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

#include "slt_common/calib_manager_node.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "slt_common/msg_utils.hpp"

namespace slt_common
{
namespace
{
bool parse_camera_info(const YAML::Node & camera, sensor_msgs::msg::CameraInfo & camera_info)
{
  if (!camera["frame_id"] || !camera["width"] || !camera["height"] ||
    !camera["intrinsics"] || !camera["distortion_coeffs"])
  {
    return false;
  }
  const std::string type = camera["type"] ? camera["type"].as<std::string>() : "";
  // radtan k1 k2 p1 p2 k3 is the plumb_bob ros uses
  if (!type.empty() && type != "pinhole_radtan") {
    return false;
  }
  auto intrinsics = camera["intrinsics"].as<std::vector<double>>();
  if (intrinsics.size() < 4) {
    return false;
  }
  const double fx = intrinsics[0];
  const double fy = intrinsics[1];
  const double cx = intrinsics[2];
  const double cy = intrinsics[3];
  camera_info.header.frame_id = camera["frame_id"].as<std::string>();
  camera_info.width = camera["width"].as<int>();
  camera_info.height = camera["height"].as<int>();
  camera_info.distortion_model = "plumb_bob";
  camera_info.d = camera["distortion_coeffs"].as<std::vector<double>>();
  camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return true;
}
}  // namespace

CalibManagerNode::CalibManagerNode(rclcpp::Node::SharedPtr node)
: node_(node)
{
  std::string calibration_config;
  node->declare_parameter("calibration_config", calibration_config);
  node->get_parameter("calibration_config", calibration_config);
  // static tf latches on /tf_static; the camera info is latched too, so a late consumer still
  // gets the one-shot calibration
  static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  camera_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::QoS(1).transient_local());
  if (calibration_config.empty() || !std::filesystem::exists(calibration_config)) {
    RCLCPP_FATAL(
      node->get_logger(), "calibration_config is invalid: %s", calibration_config.c_str());
    return;
  }
  if (!load_calibration(calibration_config)) {
    RCLCPP_FATAL(
      node->get_logger(), "failed to load calibration from: %s", calibration_config.c_str());
    return;
  }
  publish_extrinsics();
  publish_camera_info();
}

bool CalibManagerNode::load_calibration(const std::string & config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);
  // transforms: 'frame_id -> child_frame_id' is frame_id_T_child_frame_id
  if (config["transforms"]) {
    for (const auto & node : config["transforms"]) {
      const auto & entry = node.second;
      if (!entry["frame_id"] || !entry["child_frame_id"] || !entry["translation"] ||
        !entry["rotation_xyzw"])
      {
        RCLCPP_ERROR(node_->get_logger(), "invalid transform entry");
        return false;
      }
      auto translation = entry["translation"].as<std::vector<double>>();
      auto rotation = entry["rotation_xyzw"].as<std::vector<double>>();
      if (translation.size() != 3 || rotation.size() != 4) {
        RCLCPP_ERROR(node_->get_logger(), "invalid transform entry");
        return false;
      }
      StaticTransform transform;
      transform.frame_id = entry["frame_id"].as<std::string>();
      transform.child_frame_id = entry["child_frame_id"].as<std::string>();
      transform.pose.setIdentity();
      transform.pose.block<3, 1>(0, 3) =
        Eigen::Vector3d(translation[0], translation[1], translation[2]);
      transform.pose.block<3, 3>(0, 0) =
        Eigen::Quaterniond(rotation[3], rotation[0], rotation[1], rotation[2])
        .normalized().toRotationMatrix();
      extrinsics_.push_back(transform);
    }
  }
  // the first camera becomes the camera info; the imu entries are ignored
  if (config["cameras"] && config["cameras"].size() > 0) {
    const auto & camera = config["cameras"].begin()->second;
    if (!parse_camera_info(camera, camera_info_)) {
      RCLCPP_ERROR(node_->get_logger(), "invalid camera entry");
      return false;
    }
    has_camera_info_ = true;
  }
  if (extrinsics_.empty() && !has_camera_info_) {
    return false;
  }
  RCLCPP_INFO(
    node_->get_logger(), "loaded %lu transforms, camera info: %s", extrinsics_.size(),
    has_camera_info_ ? "yes" : "no");
  return true;
}

void CalibManagerNode::publish_extrinsics()
{
  std::vector<geometry_msgs::msg::TransformStamped> messages;
  for (const auto & transform : extrinsics_) {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = node_->now();
    message.header.frame_id = transform.frame_id;
    message.child_frame_id = transform.child_frame_id;
    message.transform = to_transform_msg(transform.pose);
    messages.push_back(message);
  }
  if (!messages.empty()) {
    static_tf_pub_->sendTransform(messages);
    RCLCPP_INFO(node_->get_logger(), "published %lu static transforms", messages.size());
  }
}

void CalibManagerNode::publish_camera_info()
{
  if (!has_camera_info_) {
    return;
  }
  camera_info_.header.stamp = node_->now();
  camera_info_pub_->publish(camera_info_);
  RCLCPP_INFO(
    node_->get_logger(), "published camera info: %dx%d, fx %f fy %f cx %f cy %f",
    camera_info_.width, camera_info_.height, camera_info_.k[0], camera_info_.k[4],
    camera_info_.k[2], camera_info_.k[5]);
}
}  // namespace slt_common
