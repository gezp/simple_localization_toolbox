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

#include "slt_bringup_m3dgr/replay_node.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "slt_common/msg_utils.hpp"

namespace slt_bringup_m3dgr
{

ReplayNode::ReplayNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // parameters
  std::string calibration_config, gt_path;
  node->declare_parameter("calibration_config", calibration_config);
  node->declare_parameter("gt_path", gt_path);
  node->declare_parameter("rate", rate_);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("rebuild_gt_rotation", rebuild_gt_rotation_);
  node->get_parameter("calibration_config", calibration_config);
  node->get_parameter("gt_path", gt_path);
  node->get_parameter("rate", rate_);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("rebuild_gt_rotation", rebuild_gt_rotation_);
  // pub & sub
  static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  gt_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "m3dgr/ground_truth/odom", "map", "base_link", 10);
  if (publish_tf_) {
    gt_pub_->set_tf_broadcaster(tf_pub_);
  }
  // extrinsics -> static tf
  if (!load_extrinsics(calibration_config)) {
    RCLCPP_FATAL(node->get_logger(), "failed to load extrinsics from: %s",
                 calibration_config.c_str());
    return;
  }
  // ground truth (load first: extrinsics tf stamp uses gt first frame time)
  if (!gt_path.empty() && !load_ground_truth(gt_path)) {
    RCLCPP_FATAL(node->get_logger(), "failed to load ground truth from: %s", gt_path.c_str());
    return;
  }
  if (rebuild_gt_rotation_) {
    rebuild_gt_rotation();
  }
  run_thread_ = std::thread([this]() {
    publish_extrinsics();
    replay_ground_truth();
  });
}

ReplayNode::~ReplayNode()
{
  exit_ = true;
  if (run_thread_.joinable()) {
    run_thread_.join();
  }
}

bool ReplayNode::load_extrinsics(const std::string & config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);
  if (!config["extrinsics"]) {
    return false;
  }
  for (const auto & entry : config["extrinsics"]) {
    StaticTransform transform;
    transform.frame_id = entry["frame_id"].as<std::string>();
    transform.child_frame_id = entry["child_frame_id"].as<std::string>();
    auto translation = entry["translation"].as<std::vector<double>>();
    auto rotation = entry["rotation"].as<std::vector<double>>();
    if (translation.size() != 3 || rotation.size() != 9) {
      RCLCPP_ERROR(node_->get_logger(), "invalid extrinsics %s -> %s",
                   transform.frame_id.c_str(), transform.child_frame_id.c_str());
      return false;
    }
    transform.pose.setIdentity();
    transform.pose.block<3, 1>(0, 3) =
      Eigen::Vector3d(translation[0], translation[1], translation[2]);
    transform.pose.block<3, 3>(0, 0) =
      Eigen::Matrix3d(Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        rotation.data()));
    extrinsics_.push_back(transform);
  }
  RCLCPP_INFO(node_->get_logger(), "loaded %lu extrinsics", extrinsics_.size());
  return !extrinsics_.empty();
}

void ReplayNode::publish_extrinsics()
{
  std::vector<geometry_msgs::msg::TransformStamped> messages;
  for (const auto & transform : extrinsics_) {
    geometry_msgs::msg::TransformStamped message;
    message.header.frame_id = transform.frame_id;
    message.child_frame_id = transform.child_frame_id;
    message.transform = slt_common::to_transform_msg(transform.pose);
    messages.push_back(message);
  }
  static_tf_pub_->sendTransform(messages);
  RCLCPP_INFO(node_->get_logger(), "published %lu static transforms", messages.size());
}

bool ReplayNode::load_ground_truth(const std::string & gt_path)
{
  std::ifstream ifs(gt_path);
  if (!ifs) {
    return false;
  }
  // gt format: timestamp x y z qx qy qz qw (pose relative to first frame)
  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss(line);
    double time, x, y, z, qx, qy, qz, qw;
    if (!(iss >> time >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      continue;
    }
    slt_common::OdomData odom;
    odom.time = time;
    odom.pose.setIdentity();
    odom.pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    odom.pose.block<3, 3>(0, 0) = Eigen::Quaterniond(qw, qx, qy, qz).normalized().matrix();
    ground_truth_.push_back(odom);
  }
  RCLCPP_INFO(node_->get_logger(), "ground truth loaded: %lu poses, range [%lf, %lf]",
              ground_truth_.size(), ground_truth_.front().time, ground_truth_.back().time);
  return !ground_truth_.empty();
}

void ReplayNode::rebuild_gt_rotation()
{
  // Rebuild yaw (only) from trajectory:
  //   target   = heading of chord spanning +-1m of trajectory arc
  //   valid    = chord length >= half of arc (straight motion vs. drift noise)
  //   applied  = target rate-limited to +-5 deg per pose (no jumps, static holds)
  const size_t n = ground_truth_.size();
  auto pos = [this](size_t k) { return ground_truth_[k].pose.block<3, 1>(0, 3); };
  std::vector<double> arc(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    arc[i] = arc[i - 1] + (pos(i) - pos(i - 1)).head<2>().norm();
  }
  auto idx = [&](double s) {
    return std::clamp<size_t>(std::lower_bound(arc.begin(), arc.end(), s) - arc.begin(), 0, n - 1);
  };
  const double window = 1.0, max_step = 5.0 * M_PI / 180.0;
  // rate limit only applies across consecutive moving poses; after a static
  // segment (start or mid-trajectory) jump directly to current chord heading
  double yaw = 0.0;
  bool static_prev = true;
  for (size_t i = 0; i < n; ++i) {
    size_t b = idx(arc[i] - window), f = idx(arc[i] + window);
    Eigen::Vector2d d = (pos(f) - pos(b)).head<2>();
    bool moving = d.norm() > 0.1 && d.norm() >= 0.5 * (arc[f] - arc[b]);
    if (moving) {
      double target = std::atan2(d.y(), d.x());
      yaw = static_prev
              ? target
              : yaw + std::clamp(std::remainder(target - yaw, 2 * M_PI), -max_step, max_step);
    }
    static_prev = !moving;
    ground_truth_[i].pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }
}

void ReplayNode::replay_ground_truth()
{
  if (ground_truth_.empty()) {
    return;
  }
  const double start_time = ground_truth_.front().time;
  const auto t0 = std::chrono::steady_clock::now();
  for (const auto & odom : ground_truth_) {
    if (exit_) {
      return;
    }
    const double wait =
      (odom.time - start_time) / rate_ -
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (wait > 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(wait));
    }
    gt_pub_->publish(odom);
  }
  RCLCPP_INFO(node_->get_logger(), "ground truth replay done");
}

}  // namespace slt_bringup_m3dgr
