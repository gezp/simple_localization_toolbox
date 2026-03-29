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

#include "slt_lidar_odometry/lidar_odometry_node.hpp"

#include <pcl/common/transforms.h>
#include <filesystem>

#include "slt_common/sensor_data_utils.hpp"
#include "slt_common/lidar_utils.hpp"

namespace slt_lidar_odometry
{

using OdometryMethod = LidarOdometryNode::OdometryMethod;

LidarOdometryNode::LidarOdometryNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // lidar_odometry_config
  std::string lidar_odometry_config;
  node->declare_parameter("lidar_odometry_config", lidar_odometry_config);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("use_initial_pose_from_topic", use_initial_pose_from_topic_);
  node->declare_parameter("undistort_point_cloud", undistort_point_cloud_);
  node->declare_parameter("publish_undistorted_point_cloud", publish_undistorted_point_cloud_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("lidar_frame_id", lidar_frame_id_);
  node->get_parameter("lidar_odometry_config", lidar_odometry_config);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("use_initial_pose_from_topic", use_initial_pose_from_topic_);
  node->get_parameter("undistort_point_cloud", undistort_point_cloud_);
  node->get_parameter("publish_undistorted_point_cloud", publish_undistorted_point_cloud_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("lidar_frame_id", lidar_frame_id_);
  RCLCPP_INFO(node->get_logger(), "lidar_odometry_config: [%s]", lidar_odometry_config.c_str());
  if (lidar_odometry_config == "" || (!std::filesystem::exists(lidar_odometry_config))) {
    RCLCPP_FATAL(node->get_logger(), "lidar_odometry_config is invalid");
    return;
  }
  // create lidar_odometry
  YAML::Node config = YAML::LoadFile(lidar_odometry_config);
  std::string method = config["lidar_odometry_method"].as<std::string>();
  if (method == "simple") {
    odometry_method_ = OdometryMethod::Simple;
    simple_odometry_ = std::make_shared<SimpleOdometry>(config["simple"]);
  } else if (method == "loam") {
    odometry_method_ = OdometryMethod::Loam;
    loam_odometry_ = std::make_shared<LoamOdometry>(config["loam"]);
  } else {
    RCLCPP_FATAL(node->get_logger(), "unknown odometry method: %s\n", method.c_str());
    return;
  }
  bool enable = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enable);
  elapsed_time_statistics_.set_title("LidarOdometryNode");
  // sub & pub
  cloud_sub_ = std::make_shared<slt_common::CloudSubscriber>(node, "synced_cloud", 10000);
  if (use_initial_pose_from_topic_) {
    reference_odom_sub_ =
      std::make_shared<slt_common::OdometrySubscriber>(node, "reference_odom", 10000);
  }
  undistorted_scan_pub_ = std::make_shared<slt_common::CloudPublisher>(
    node, "lidar_odometry/undistorted_pointcloud", lidar_frame_id_, 100);
  current_scan_pub_ =
    std::make_shared<slt_common::CloudPublisher>(node, "lidar_odometry/current_scan", "map", 100);
  local_map_pub_ =
    std::make_shared<slt_common::CloudPublisher>(node, "lidar_odometry/local_map", "map", 100);
  loam_feature_pub_ =
    std::make_shared<slt_common::CloudPublisher>(node, "lidar_odometry/loam_feature", "map", 100);
  lidar_odom_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "lidar_odometry/odom", "map", base_frame_id_, 100);
  if (publish_tf_) {
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    lidar_odom_pub_->set_tf_broadcaster(tf_pub_);
  }
  // extrinsics
  extrinsics_manager_ = std::make_shared<slt_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // buffer
  ref_odom_buffer_ = std::make_shared<slt_common::OdomDataBuffer>(10000);
  // process loop callback
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

LidarOdometryNode::~LidarOdometryNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LidarOdometryNode::run()
{
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, lidar_frame_id_, T_base_lidar_)) {
      return false;
    }
    set_extrinsics_for_odometry(odometry_method_, T_base_lidar_);
    is_valid_extrinsics_ = true;
  }
  // read data
  cloud_sub_->parse_data(lidar_data_buffer_);
  if (lidar_data_buffer_.empty()) {
    return false;
  }
  // set initial pose for better visualization
  if (use_initial_pose_from_topic_ && !inited_) {
    bool is_old_data = false;
    double time = lidar_data_buffer_.front().time;
    if (!get_initial_pose_by_reference_odom(time, T_map_odom_, is_old_data)) {
      if (is_old_data) {
        lidar_data_buffer_.pop_front();
      }
      return false;
    }
    inited_ = true;
  }
  // process lidar data
  if (update_odometry(odometry_method_, lidar_data_buffer_.front())) {
    publish_data(odometry_method_);
  }
  lidar_data_buffer_.pop_front();
  elapsed_time_statistics_.print_all_info("update_odometry", 100);
  return true;
}

bool LidarOdometryNode::get_initial_pose_by_reference_odom(
  double time, Eigen::Matrix4d & initial_pose, bool & is_old_data)
{
  // read reference_odom data
  std::deque<slt_common::OdomData> buffer;
  reference_odom_sub_->parse_data(buffer);
  for (auto & data : buffer) {
    ref_odom_buffer_->add_data(data);
  }
  // check
  is_old_data = false;
  if (ref_odom_buffer_->size() == 0) {
    return false;
  }
  if (time < ref_odom_buffer_->get_start_time()) {
    is_old_data = true;
    std::cout << "need drop oldest lidar data" << std::endl;
    return false;
  }
  if (time > ref_odom_buffer_->get_end_time()) {
    // wait valid ref odom
    return false;
  }
  // get synced odom
  slt_common::OdomData odom;
  if (!ref_odom_buffer_->get_interpolated_data(time, odom)) {
    return false;
  }
  initial_pose = odom.pose;
  Eigen::Vector3d p = odom.pose.block<3, 1>(0, 3);
  RCLCPP_INFO(node_->get_logger(), "initialize at position: (%lf, %lf, %lf)", p.x(), p.y(), p.z());
  reference_odom_sub_.reset();
  return true;
}

void LidarOdometryNode::set_extrinsics_for_odometry(
  OdometryMethod method, const Eigen::Matrix4d & T_base_lidar)
{
  if (method == OdometryMethod::Simple) {
    simple_odometry_->set_extrinsic(T_base_lidar);
  } else if (method == OdometryMethod::Loam) {
    loam_odometry_->set_extrinsic(T_base_lidar);
  }
}

bool LidarOdometryNode::update_odometry(OdometryMethod method, slt_common::LidarData & lidar_data)
{
  elapsed_time_statistics_.tic("update_odometry");
  // undistort point cloud
  if (undistort_point_cloud_) {
    slt_common::undistort_point_cloud(lidar_data, last_twist_);
    if (publish_undistorted_point_cloud_) {
      undistorted_scan_pub_->publish(lidar_data);
    }
  }
  bool success = false;
  if (method == OdometryMethod::Simple) {
    success = simple_odometry_->update(lidar_data);
  } else if (method == OdometryMethod::Loam) {
    success = loam_odometry_->update(lidar_data);
  }
  elapsed_time_statistics_.toc("update_odometry");
  return success;
}

slt_common::OdomData LidarOdometryNode::align_odom_to_map(const slt_common::OdomData & odom)
{
  slt_common::OdomData odom_aligned;
  odom_aligned.time = odom.time;
  odom_aligned.pose = T_map_odom_ * odom.pose;
  odom_aligned.linear_velocity = T_map_odom_.block<3, 3>(0, 0) * odom.linear_velocity;
  odom_aligned.angular_velocity = T_map_odom_.block<3, 3>(0, 0) * odom.angular_velocity;
  return odom_aligned;
}

void LidarOdometryNode::publish_data(OdometryMethod method)
{
  elapsed_time_statistics_.tic("publish_data");
  slt_common::OdomData odom;
  if (method == OdometryMethod::Simple) {
    // publish odom
    odom = align_odom_to_map(simple_odometry_->get_current_odom());
    lidar_odom_pub_->publish(odom);
    // publish point cloud
    if (current_scan_pub_->has_subscribers()) {
      auto current_scan = simple_odometry_->get_current_scan();
      pcl::transformPointCloud(*current_scan, *current_scan, odom.pose);
      current_scan_pub_->publish(*current_scan);
    }
    if (simple_odometry_->has_new_local_map() && local_map_pub_->has_subscribers()) {
      auto local_map = simple_odometry_->get_local_map();
      pcl::transformPointCloud(*local_map, *local_map, T_map_odom_);
      local_map_pub_->publish(*local_map);
    }
  } else if (method == OdometryMethod::Loam) {
    // publish odom
    odom = align_odom_to_map(loam_odometry_->get_current_odom());
    lidar_odom_pub_->publish(odom);
    // publish point cloud
    if (current_scan_pub_->has_subscribers()) {
      auto current_scan = loam_odometry_->get_current_scan();
      pcl::transformPointCloud(*current_scan, *current_scan, odom.pose);
      current_scan_pub_->publish(*current_scan);
    }
    if (loam_feature_pub_->has_subscribers()) {
      auto feature_scan = loam_odometry_->get_feature_scan();
      pcl::transformPointCloud(*feature_scan, *feature_scan, odom.pose);
      loam_feature_pub_->publish(*feature_scan);
    }
  }
  last_twist_ = slt_common::get_twist_from_odom(odom);
  elapsed_time_statistics_.toc("publish_data");
}

}  // namespace slt_lidar_odometry
