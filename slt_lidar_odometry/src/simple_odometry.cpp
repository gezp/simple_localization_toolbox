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

#include "slt_lidar_odometry/simple_odometry.hpp"

#include <pcl/common/transforms.h>

#include "slt_common/sensor_data_utils.hpp"
#include "slt_common/lidar_utils.hpp"

namespace slt_lidar_odometry
{
SimpleOdometry::SimpleOdometry(const YAML::Node & config)
{
  registration_factory_ = std::make_shared<slt_common::PointCloudRegistrationFactory>();
  //
  local_frame_num_ = config["local_frame_num"].as<int>();
  key_frame_distance_ = config["key_frame_distance"].as<float>();
  key_frame_angle_ = config["key_frame_angle"].as<float>() * M_PI / 180.0;
  // init registration and filter
  registration_ = registration_factory_->create(config["registration"]);
  using VoxelFilter = slt_common::VoxelFilter;
  local_map_filter_ = std::make_shared<VoxelFilter>(config["local_map_filter"]);
  current_scan_filter_ = std::make_shared<VoxelFilter>(config["current_scan_filter"]);
  display_filter_ = std::make_shared<VoxelFilter>(config["display_filter"]);
  // print info
  std::cout << "cloud registration:" << std::endl;
  registration_->print_info();
  std::cout << "local_map filter:" << std::endl;
  local_map_filter_->print_info();
  std::cout << "current_scan filter:" << std::endl;
  current_scan_filter_->print_info();
  std::cout << "display filter:" << std::endl;
  display_filter_->print_info();
}

void SimpleOdometry::set_extrinsic(const Eigen::Matrix4d & T_base_lidar)
{
  T_base_lidar_ = T_base_lidar;
  T_lidar_base_ = T_base_lidar.inverse();
}

bool SimpleOdometry::update(const slt_common::LidarData & lidar_data)
{
  has_new_local_map_ = false;
  current_frame_.time = lidar_data.time;
  current_frame_.point_cloud = to_pointcloud_xyz(lidar_data);
  if (key_frames_.empty()) {
    // initialize the first frame
    current_frame_.pose = T_base_lidar_;
  } else {
    // scan to map matching
    Eigen::Matrix4d predict_pose = Eigen::Matrix4d::Identity();
    if (!get_initial_pose_by_history(predict_pose)) {
      std::cout << "failed to get predict pose by history" << std::endl;
    }
    match_scan_to_map(predict_pose, current_frame_.pose);
  }
  // add into history_poses_
  update_history_pose(current_frame_.time, current_frame_.pose);
  // check if add new frame and update local map
  if (check_new_key_frame(current_frame_.pose)) {
    has_new_local_map_ = true;
    // update local map
    update_local_map();
    // update target of registration
    registration_->set_target(local_map_);
  }
  return true;
}

slt_common::OdomData SimpleOdometry::get_current_odom()
{
  slt_common::OdomData odom;
  odom.time = current_frame_.time;
  odom.pose = current_frame_.pose * T_lidar_base_;
  if (history_poses_.size() >= 2) {
    auto & pose1 = history_poses_[history_poses_.size() - 2];
    auto & pose2 = history_poses_[history_poses_.size() - 1];
    auto twist = slt_common::estimate_twist_by_pose(pose1, pose2);
    odom.linear_velocity = twist.linear_velocity;
    odom.angular_velocity = twist.angular_velocity;
  }
  return odom;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleOdometry::get_current_scan()
{
  return display_filter_->apply(current_frame_.point_cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SimpleOdometry::get_local_map()
{
  return display_filter_->apply(local_map_);
}

bool SimpleOdometry::has_new_local_map()
{
  return has_new_local_map_;
}

bool SimpleOdometry::update_history_pose(double time, const Eigen::Matrix4d & pose)
{
  slt_common::PoseData pose_data;
  pose_data.time = time;
  pose_data.pose = pose;
  history_poses_.push_back(pose_data);
  if (history_poses_.size() > 100) {
    history_poses_.pop_front();
  }
  return true;
}

bool SimpleOdometry::get_initial_pose_by_history(Eigen::Matrix4d & initial_pose)
{
  if (history_poses_.empty()) {
    return false;
  }
  if (history_poses_.size() == 1) {
    initial_pose = history_poses_.back().pose;
  } else {
    Eigen::Matrix4d last_pose1 = history_poses_[history_poses_.size() - 2].pose;
    Eigen::Matrix4d last_pose2 = history_poses_.back().pose;
    Eigen::Matrix4d step_pose = last_pose1.inverse() * last_pose2;
    initial_pose = last_pose2 * step_pose;
  }
  return true;
}

bool SimpleOdometry::check_new_key_frame(const Eigen::Matrix4d & pose)
{
  if (key_frames_.empty()) {
    return true;
  }
  Eigen::Vector3d dis = key_frames_.back().pose.block<3, 1>(0, 3) - pose.block<3, 1>(0, 3);
  if (dis.norm() > key_frame_distance_) {
    return true;
  }
  Eigen::Matrix3d R_rel =
    key_frames_.back().pose.block<3, 3>(0, 0).transpose() * pose.block<3, 3>(0, 0);
  if (Eigen::AngleAxisd(R_rel).angle() > key_frame_angle_) {
    return true;
  }
  return false;
}

bool SimpleOdometry::update_local_map()
{
  // add new key frame
  key_frames_.push_back(current_frame_);
  // move window for local map
  while (key_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    key_frames_.pop_front();
  }
  // update local map
  local_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = 0; i < key_frames_.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*key_frames_.at(i).point_cloud, *cloud, key_frames_.at(i).pose);
    *local_map_ += *cloud;
  }
  // downsample for local map if too much key frames
  if (key_frames_.size() >= 10) {
    local_map_ = local_map_filter_->apply(local_map_);
  }
  return true;
}

bool SimpleOdometry::match_scan_to_map(
  const Eigen::Matrix4d & predict_pose, Eigen::Matrix4d & final_pose)
{
  // downsample current lidar point cloud
  auto filtered_cloud = current_scan_filter_->apply(current_frame_.point_cloud);
  // matching
  registration_->match(filtered_cloud, predict_pose);
  // result
  final_pose = registration_->get_final_pose();
  return true;
}

}  // namespace slt_lidar_odometry
