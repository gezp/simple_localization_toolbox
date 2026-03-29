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

#include "slt_graph_locator/sliding_window_node.hpp"

#include "slt_common/msg_utils.hpp"

namespace slt_graph_locator
{

SlidingWindowNode::SlidingWindowNode(rclcpp::Node::SharedPtr node)
{
  std::string config_file;
  node->declare_parameter("config_file", config_file);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("imu_frame_id", imu_frame_id_);
  node->get_parameter("config_file", config_file);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("imu_frame_id", imu_frame_id_);
  std::cout << "config file path:" << config_file << std::endl;
  // sub&pub
  lidar_pose_sub_ = std::make_shared<slt_common::OdometrySubscriber>(
    node, "localization/lidar/pose", 10000);
  gnss_pose_sub_ =
    std::make_shared<slt_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  raw_imu_sub_ =
    std::make_shared<slt_common::ImuSubscriber>(node, "/kitti/oxts/imu/extract", 100000);
  optimized_odom_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "localization/fused/pose", "map", base_frame_id_, 100);
  // tf:
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  optimized_odom_pub_->set_tf_broadcaster(tf_pub_);
  // extrinsics
  extrinsics_manager_ = std::make_shared<slt_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // sliding_window:
  YAML::Node config_node = YAML::LoadFile(config_file);
  sliding_window_ = std::make_shared<SlidingWindow>();
  sliding_window_->init_with_config(config_node);
  // thread
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        if (!run()) {
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(5ms);
        }
      }
    });
}

SlidingWindowNode::~SlidingWindowNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool SlidingWindowNode::run()
{
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, imu_frame_id_, T_base_imu_)) {
      return false;
    }
    sliding_window_->set_extrinsic(T_base_imu_);
    is_valid_extrinsics_ = true;
  }
  // read data
  read_data();
  // process data
  if (!raw_imu_data_buffer_.empty()) {
    sliding_window_->add_imu_data(raw_imu_data_buffer_.front());
    raw_imu_data_buffer_.pop_front();
  }
  if (!gnss_pose_buffer_.empty()) {
    current_gnss_pose_ = gnss_pose_buffer_.front();
    sliding_window_->add_gnss_pose(current_gnss_pose_);
    gnss_pose_buffer_.pop_front();
  }
  if (!lidar_pose_buffer_.empty()) {
    current_lidar_pose_ = lidar_pose_buffer_.front();
    sliding_window_->add_lidar_pose(current_lidar_pose_);
    lidar_pose_buffer_.pop_front();
  }
  if (sliding_window_->update()) {
    publish_data();
    return true;
  }
  return false;
}

bool SlidingWindowNode::read_data()
{
  raw_imu_sub_->parse_data(raw_imu_data_buffer_);
  lidar_pose_sub_->parse_data(lidar_pose_buffer_);
  gnss_pose_sub_->parse_data(gnss_pose_buffer_);
  return true;
}

bool SlidingWindowNode::publish_data()
{
  // get odom
  auto odom = sliding_window_->get_current_odom();
  // publish fusion odometry
  optimized_odom_pub_->publish(odom);
  return true;
}

}  // namespace slt_graph_locator
