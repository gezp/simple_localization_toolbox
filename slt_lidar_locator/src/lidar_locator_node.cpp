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

#include "slt_lidar_locator/lidar_locator_node.hpp"

#include <filesystem>

#include "slt_common/msg_utils.hpp"
#include "slt_common/sensor_data_utils.hpp"
#include "slt_common/lidar_utils.hpp"

namespace slt_lidar_locator
{
LidarLocalizationNode::LidarLocalizationNode(rclcpp::Node::SharedPtr node)
{
  std::string slt_lidar_locator_config;
  std::string data_path;
  node->declare_parameter("slt_lidar_locator_config", slt_lidar_locator_config);
  node->declare_parameter("data_path", data_path);
  node->declare_parameter("undistort_point_cloud", undistort_point_cloud_);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("lidar_frame_id", lidar_frame_id_);
  node->get_parameter("slt_lidar_locator_config", slt_lidar_locator_config);
  node->get_parameter("data_path", data_path);
  node->get_parameter("undistort_point_cloud", undistort_point_cloud_);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("lidar_frame_id", lidar_frame_id_);
  RCLCPP_INFO(
    node->get_logger(), "slt_lidar_locator_config: [%s]", slt_lidar_locator_config.c_str());
  RCLCPP_INFO(node->get_logger(), "data_path: [%s]", data_path.c_str());
  if (slt_lidar_locator_config == "" || (!std::filesystem::exists(slt_lidar_locator_config))) {
    RCLCPP_FATAL(node->get_logger(), "slt_lidar_locator_config is invalid");
    return;
  }
  if (data_path == "" || (!std::filesystem::is_directory(data_path))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid");
    return;
  }
  // subscriber
  cloud_sub_ = std::make_shared<slt_common::CloudSubscriber>(node, "synced_cloud", 10000);
  gnss_data_sub_ =
    std::make_shared<slt_common::GnssSubscriber>(node, "/kitti/gnss_data", 10000);
  gnss_odom_sub_ =
    std::make_shared<slt_common::OdometrySubscriber>(node, "synced_gnss/pose", 10000);
  // publisher
  global_map_pub_ = std::make_shared<slt_common::CloudPublisher>(
    node, "slt_lidar_locator/global_map", "map", 100);
  local_map_pub_ = std::make_shared<slt_common::CloudPublisher>(
    node, "slt_lidar_locator/local_map", "map", 100);
  current_scan_pub_ = std::make_shared<slt_common::CloudPublisher>(
    node, "slt_lidar_locator/current_scan", "map", 100);
  lidar_pose_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "localization/lidar/pose", "map", base_frame_id_, 100);
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // extrinsics tool
  extrinsics_manager_ = std::make_shared<slt_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  std::cout << "-----------------Init Lidar Localization-------------------" << std::endl;
  slt_lidar_locator_ = std::make_shared<LidarLocalization>();
  slt_lidar_locator_->init_config(slt_lidar_locator_config, data_path);
  // process loop flow
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        if (!run()) {
          using namespace std::chrono_literals;
          std::this_thread::sleep_for(10ms);
        }
      }
    });
}

LidarLocalizationNode::~LidarLocalizationNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LidarLocalizationNode::run()
{
  // for global map visualization
  static bool global_map_published = false;
  if (!global_map_published && global_map_pub_->has_subscribers()) {
    auto global_map = slt_lidar_locator_->get_global_map();
    global_map_pub_->publish(*global_map);
    global_map_published = true;
  }
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, lidar_frame_id_, T_base_lidar_)) {
      return false;
    }
    slt_lidar_locator_->set_extrinsic(T_base_lidar_);
    is_valid_extrinsics_ = true;
  }
  // read data
  read_data();
  // process gnss data
  while (!gnss_data_buffer_.empty()) {
    auto current_gnss_data = gnss_data_buffer_.front();
    slt_lidar_locator_->add_gnss_data(current_gnss_data);
    gnss_data_buffer_.pop_front();
  }
  // process gnss odometry
  while (!gnss_odom_buffer_.empty()) {
    auto current_gnss_odom = gnss_odom_buffer_.front();
    slt_lidar_locator_->add_gnss_odom(current_gnss_odom);
    gnss_odom_buffer_.pop_front();
  }
  // process lidar data
  if (!lidar_data_buffer_.empty()) {
    auto current_lidar_data = lidar_data_buffer_.front();
    // undistort point cloud
    if (undistort_point_cloud_) {
      slt_common::undistort_point_cloud(current_lidar_data, last_twist_);
    }
    if (slt_lidar_locator_->update(current_lidar_data)) {
      publish_data();
    }
    lidar_data_buffer_.pop_front();
    return true;
  }
  return false;
}

bool LidarLocalizationNode::read_data()
{
  cloud_sub_->parse_data(lidar_data_buffer_);
  gnss_data_sub_->parse_data(gnss_data_buffer_);
  gnss_odom_sub_->parse_data(gnss_odom_buffer_);
  return true;
}

bool LidarLocalizationNode::publish_data()
{
  // publish lidar pose
  auto odom = slt_lidar_locator_->get_current_odom();
  lidar_pose_pub_->publish(odom);
  last_twist_ = slt_common::get_twist_from_odom(odom);
  // publish tf
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = slt_common::to_ros_time(odom.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_frame_id_;
    msg.transform = slt_common::to_transform_msg(odom.pose);
    tf_pub_->sendTransform(msg);
  }
  // puslish point cloud
  if (current_scan_pub_->has_subscribers()) {
    current_scan_pub_->publish(*slt_lidar_locator_->get_current_scan());
  }
  if (slt_lidar_locator_->has_new_local_map() && local_map_pub_->has_subscribers()) {
    local_map_pub_->publish(*slt_lidar_locator_->get_local_map());
  }
  return true;
}

}  // namespace slt_lidar_locator
