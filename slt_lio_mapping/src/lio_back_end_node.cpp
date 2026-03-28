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

#include "slt_lio_mapping/lio_back_end_node.hpp"

#include <filesystem>
#include "slt_common/msg_utils.hpp"

namespace slt_lio_mapping
{
LioBackEndNode::LioBackEndNode(rclcpp::Node::SharedPtr node)
{
  std::string back_end_config;
  std::string data_path;
  node->declare_parameter("back_end_config", back_end_config);
  node->declare_parameter("data_path", data_path);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("lidar_frame_id", lidar_frame_id_);
  node->declare_parameter("imu_frame_id", imu_frame_id_);
  node->get_parameter("back_end_config", back_end_config);
  node->get_parameter("data_path", data_path);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("lidar_frame_id", lidar_frame_id_);
  node->get_parameter("imu_frame_id", imu_frame_id_);
  RCLCPP_INFO(node->get_logger(), "back_end_config: [%s]", back_end_config.c_str());
  RCLCPP_INFO(node->get_logger(), "data_path: [%s]", data_path.c_str());
  if (back_end_config == "" || (!std::filesystem::exists(back_end_config))) {
    RCLCPP_FATAL(node->get_logger(), "back_end_config is invalid");
    return;
  }
  if (data_path == "" || (!std::filesystem::is_directory(data_path))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid");
    return;
  }
  //
  std::cout << "-----------------Init Backend-------------------" << std::endl;
  back_end_ = std::make_shared<LioBackEnd>();
  if (!back_end_->init_config(back_end_config, data_path)) {
    std::cout << "faild to init backend!" << std::endl;
    return;
  }
  // sub & pub
  cloud_sub_ = std::make_shared<slt_common::CloudSubscriber>(node, "synced_cloud", 100000);
  gnss_pose_sub_ =
    std::make_shared<slt_common::OdometrySubscriber>(node, "synced_gnss/pose", 100000);
  lidar_odom_sub_ =
    std::make_shared<slt_common::OdometrySubscriber>(node, "lidar_odometry/odom", 100000);
  loop_candidate_sub_ =
    std::make_shared<slt_common::LoopCandidateSubscriber>(node, "loop_candidate", 100000);
  raw_imu_sub_ =
    std::make_shared<slt_common::ImuSubscriber>(node, "/kitti/oxts/imu/extract", 1000000);
  key_frames_pub_ =
    std::make_shared<slt_common::LidarFramesPublisher>(node, "key_frames", 100);
  optimized_path_pub_ =
    std::make_shared<slt_common::PathPublisher>(node, "optimized_path", "map", 100);
  optimized_odom_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "optimized_pose", "map", base_frame_id_, 100);
  global_map_pub_ =
    std::make_shared<slt_common::CloudPublisher>(node, "global_map", "map", 100);
  // tf
  tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  // extrinsics
  extrinsics_manager_ = std::make_shared<slt_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // srv
  optimize_map_srv_ = node->create_service<slt_interface::srv::OptimizeMap>(
    "optimize_map",
    [&](
      const slt_interface::srv::OptimizeMap::Request::SharedPtr /*request*/,
      const slt_interface::srv::OptimizeMap::Response::SharedPtr response) {
      need_optimize_map_ = true;
      response->succeed = true;
    });
  save_map_srv_ = node->create_service<slt_interface::srv::SaveMap>(
    "save_map", [this](
      const slt_interface::srv::SaveMap::Request::SharedPtr /*request*/,
      slt_interface::srv::SaveMap::Response::SharedPtr response) {
      save_map_flag_ = true;
      response->succeed = true;
    });
  // process loop flow callback
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        run();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
      }
    });
}

LioBackEndNode::~LioBackEndNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LioBackEndNode::run()
{
  // get extrinsics
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, imu_frame_id_, T_base_imu_)) {
      return false;
    }
    if (!extrinsics_manager_->lookup(lidar_frame_id_, imu_frame_id_, T_lidar_imu_)) {
      return false;
    }
    if (!extrinsics_manager_->lookup(lidar_frame_id_, base_frame_id_, T_lidar_base_)) {
      return false;
    }
    back_end_->set_extrinsic(T_base_imu_, T_lidar_imu_);
    is_valid_extrinsics_ = true;
  }
  if (need_optimize_map_) {
    force_optimize();
    need_optimize_map_ = false;
  }
  if (save_map_flag_) {
    back_end_->save_map();
    save_map_flag_ = false;
  }
  // read data
  read_data();
  // add loop candidate for loop closure
  while (!loop_candidate_data_buff_.empty()) {
    back_end_->add_loop_candidate(loop_candidate_data_buff_.front());
    loop_candidate_data_buff_.pop_front();
  }
  // add imu data
  while (!imu_raw_data_buff_.empty()) {
    back_end_->add_imu_data(imu_raw_data_buff_.front());
    imu_raw_data_buff_.pop_front();
  }
  // add gnss odom
  while (!gnss_pose_data_buff_.empty()) {
    back_end_->add_gnss_odom(gnss_pose_data_buff_.front());
    gnss_pose_data_buff_.pop_front();
  }
  // update with lidar data
  if (has_data() && valid_data()) {
    if (back_end_->update(current_lidar_data_, current_lidar_odom_data_)) {
      publish_data();
    }
  }
  return true;
}

bool LioBackEndNode::force_optimize()
{
  back_end_->optimize();
  // publish optimized key frames for loop closure
  key_frames_pub_->publish(back_end_->get_key_frames());
  // publish optimized key frames
  optimized_path_pub_->publish(back_end_->get_key_frames());
  // publish global map
  if (back_end_->has_new_optimized() && global_map_pub_->has_subscribers()) {
    global_map_pub_->publish(*back_end_->get_global_map());
  }
  return true;
}

bool LioBackEndNode::read_data()
{
  cloud_sub_->parse_data(lidar_data_buff_);
  gnss_pose_sub_->parse_data(gnss_pose_data_buff_);
  lidar_odom_sub_->parse_data(lidar_odom_data_buff_);
  loop_candidate_sub_->parse_data(loop_candidate_data_buff_);
  raw_imu_sub_->parse_data(imu_raw_data_buff_);
  return true;
}

bool LioBackEndNode::has_data()
{
  if (lidar_data_buff_.size() == 0) {
    return false;
  }
  if (lidar_odom_data_buff_.size() == 0) {
    return false;
  }
  return true;
}

bool LioBackEndNode::valid_data()
{
  current_lidar_data_ = lidar_data_buff_.front();
  current_lidar_odom_data_ = lidar_odom_data_buff_.front();
  double diff_laser_time = current_lidar_data_.time - current_lidar_odom_data_.time;
  if (diff_laser_time < -0.05) {
    lidar_data_buff_.pop_front();
    return false;
  }
  if (diff_laser_time > 0.05) {
    lidar_odom_data_buff_.pop_front();
    return false;
  }
  lidar_data_buff_.pop_front();
  lidar_odom_data_buff_.pop_front();
  return true;
}

bool LioBackEndNode::publish_data()
{
  // publish odom
  auto odom = back_end_->get_current_odom();
  optimized_odom_pub_->publish(odom);
  if (publish_tf_) {
    // publish tf
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = slt_common::to_ros_time(odom.time);
    msg.header.frame_id = "map";
    msg.child_frame_id = base_frame_id_;
    msg.transform = slt_common::to_transform_msg(odom.pose);
    tf_pub_->sendTransform(msg);
  }
  // publish new key frame
  if (back_end_->has_new_key_frame()) {
    // publish optimized key frames for loop closure
    key_frames_pub_->publish(back_end_->get_key_frames());
    // publish optimized key frames
    optimized_path_pub_->publish(back_end_->get_key_frames(), T_lidar_base_);
    // publish global map
    if (back_end_->has_new_optimized() && global_map_pub_->has_subscribers()) {
      global_map_pub_->publish(*back_end_->get_global_map());
    }
  }
  return true;
}

}  // namespace slt_lio_mapping
