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

#include "lidar_localization/lidar_localization.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <cmath>

#include "localization_common/sensor_data_utils.hpp"

namespace lidar_localization
{
LidarLocalization::LidarLocalization()
{
  registration_factory_ = std::make_shared<localization_common::CloudRegistrationFactory>();
}

bool LidarLocalization::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_data_path
  data_path_ = data_path;
  map_path_ = data_path_ + "/map/filtered_map.pcd";
  scan_context_path_ = data_path_ + "/scan_context";
  // gloabl localization
  init_global_localization_config(config_node["global_localization"]);
  // init registration
  registration_ = registration_factory_->create(config_node["registration"]);
  // init filter
  using BoxFilter = localization_common::BoxFilter;
  using VoxelFilter = localization_common::VoxelFilter;
  box_filter_ = std::make_shared<BoxFilter>(config_node["roi_filter"]);
  current_scan_filter_ = std::make_shared<VoxelFilter>(config_node["current_scan_filter"]);
  local_map_filter_ = std::make_shared<VoxelFilter>(config_node["local_map_filter"]);
  display_filter_ = std::make_shared<VoxelFilter>(config_node["display_filter"]);
  // init global map
  global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::io::loadPCDFile(map_path_, *global_map_);
  std::cout << "global map size:" << global_map_->points.size() << std::endl;
  // init buffer
  gnss_odom_buffer_ = std::make_shared<localization_common::OdomDataBuffer>(1000);
  // print info
  std::cout << "cloud registration:" << std::endl;
  registration_->print_info();
  std::cout << "local_map roi filter:" << std::endl;
  box_filter_->print_info();
  std::cout << "current_scan downsample filter:" << std::endl;
  current_scan_filter_->print_info();
  std::cout << "local_map downsample filter:" << std::endl;
  local_map_filter_->print_info();
  std::cout << "visualization filter:" << std::endl;
  display_filter_->print_info();
  return true;
}

void LidarLocalization::set_extrinsic(const Eigen::Matrix4d & T_base_lidar)
{
  T_base_lidar_ = T_base_lidar;
  T_lidar_base_ = T_base_lidar.inverse();
}

bool LidarLocalization::add_gnss_data(const localization_common::GnssData & gnss_data)
{
  gnss_data_buffer_.push_back(gnss_data);
  if (gnss_data_buffer_.size() > 100) {
    gnss_data_buffer_.pop_front();
  }
  return true;
}

bool LidarLocalization::add_gnss_odom(const localization_common::OdomData & gnss_odom)
{
  gnss_odom_buffer_->add_data(gnss_odom);
  return true;
}

bool LidarLocalization::update(
  const localization_common::LidarData<localization_common::PointXYZIRT> & lidar_data)
{
  has_new_local_map_ = false;
  current_lidar_data_.time = lidar_data.time;
  current_lidar_data_.point_cloud =
    pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*lidar_data.point_cloud, *current_lidar_data_.point_cloud);
  // remove invalid measurements
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(
    *current_lidar_data_.point_cloud, *current_lidar_data_.point_cloud, indices);
  // initialize if not
  if (!has_inited_) {
    if (init_global_localization()) {
      history_frames_.push_back(current_lidar_frame_);
      has_new_local_map_ = true;
      has_inited_ = true;
      return true;
    } else {
      return false;
    }
  }
  // scan to map macthing
  Eigen::Matrix4d predict_pose = Eigen::Matrix4d::Identity();
  if (!get_initial_pose_by_history(predict_pose)) {
    std::cout << "failed to get predict pose by history" << std::endl;
  }
  match_scan_to_map(predict_pose);
  // add into lidar frame history
  history_frames_.push_back(current_lidar_frame_);
  if (history_frames_.size() > 10) {
    history_frames_.pop_front();
  }
  // check if update local map
  if (check_new_local_map(current_lidar_frame_.pose)) {
    const Eigen::Vector3d & position = current_lidar_frame_.pose.block<3, 1>(0, 3);
    update_local_map(position);
    has_new_local_map_ = true;
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarLocalization::get_global_map()
{
  return display_filter_->apply(global_map_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarLocalization::get_local_map()
{
  return display_filter_->apply(local_map_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarLocalization::get_current_scan()
{
  auto filtered_cloud = display_filter_->apply(current_lidar_data_.point_cloud);
  pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, current_lidar_frame_.pose);
  return filtered_cloud;
}

localization_common::OdomData LidarLocalization::get_current_odom()
{
  localization_common::OdomData odom;
  odom.time = current_lidar_frame_.time;
  odom.pose = current_lidar_frame_.pose * T_lidar_base_;
  if (history_frames_.size() >= 2) {
    auto & frame1 = history_frames_[history_frames_.size() - 2];
    auto & frame2 = history_frames_[history_frames_.size() - 1];
    auto twist = localization_common::estimate_twist_by_pose(
      frame1.pose, frame2.pose, frame1.time,
      frame2.time);
    odom.linear_velocity = twist.linear_velocity;
    odom.angular_velocity = twist.angular_velocity;
  }
  return odom;
}

bool LidarLocalization::has_new_local_map()
{
  return has_new_local_map_;
}

bool LidarLocalization::init_global_localization_config(const YAML::Node & config_node)
{
  use_scan_context_ = config_node["use_scan_context"].as<bool>();
  use_gnss_odometry_ = config_node["use_gnss_odometry"].as<bool>();
  use_gnss_data_ = config_node["use_gnss_data"].as<bool>();
  gnss_odometry_time_threshold_ = config_node["gnss_odometry_time_threshold"].as<double>();
  gnss_data_time_threshold_ = config_node["gnss_data_time_threshold"].as<double>();
  // coarse matching
  coarse_matching_yaw_count_ = config_node["coarse_matching_yaw_count"].as<size_t>();
  coarse_matching_error_threshold_ = config_node["coarse_matching_error_threshold"].as<double>();
  coarse_registration_ = registration_factory_->create(config_node["coarse_registration"]);
  using VoxelFilter = localization_common::VoxelFilter;
  coarse_voxel_filter_ = std::make_shared<VoxelFilter>(config_node["coarse_downsample_filter"]);
  // scan context
  if (use_scan_context_) {
    scan_context_manager_ =
      std::make_shared<scan_context::ScanContextManager>(config_node["scan_context"]);
    scan_context_manager_->load(scan_context_path_);
  }
  return true;
}

bool LidarLocalization::check_new_local_map(const Eigen::Matrix4d & pose)
{
  auto min_point = box_filter_->get_min_point();
  auto max_point = box_filter_->get_max_point();
  for (int i = 0; i < 3; i++) {
    if (fabs(pose(i, 3) - min_point(i)) > 50.0 && fabs(pose(i, 3) - max_point(i)) > 50.0) {
      continue;
    }
    return true;
  }
  return false;
}

bool LidarLocalization::update_local_map(const Eigen::Vector3d & position)
{
  // use ROI filtering for local map
  box_filter_->set_origin(position);
  local_map_ = box_filter_->apply(global_map_);
  // downsample local map
  local_map_ = local_map_filter_->apply(local_map_);
  // set registration target
  registration_->set_target(local_map_);
  return true;
}

bool LidarLocalization::match_scan_to_map(const Eigen::Matrix4d & predict_pose)
{
  // downsample current lidar point cloud
  auto filtered_cloud = current_scan_filter_->apply(current_lidar_data_.point_cloud);
  // matching
  registration_->match(filtered_cloud, predict_pose);
  // result
  current_lidar_frame_.time = current_lidar_data_.time;
  current_lidar_frame_.pose = registration_->get_final_pose();
  return true;
}

bool LidarLocalization::get_initial_pose_by_history(Eigen::Matrix4d & initial_pose)
{
  if (history_frames_.empty()) {
    return false;
  }
  if (history_frames_.size() == 1) {
    initial_pose = history_frames_.back().pose;
  } else {
    Eigen::Matrix4d last_pose1 = history_frames_[history_frames_.size() - 2].pose;
    Eigen::Matrix4d last_pose2 = history_frames_.back().pose;
    Eigen::Matrix4d step_pose = last_pose1.inverse() * last_pose2;
    initial_pose = last_pose2 * step_pose;
  }
  return true;
}

bool LidarLocalization::get_initial_pose_by_coarse_position(
  const Eigen::Vector3d & coarse_position, Eigen::Matrix4d & initial_pose)
{
  double final_error = coarse_matching_error_threshold_ + 10000;
  // get local map
  box_filter_->set_origin(coarse_position);
  auto local_map = box_filter_->apply(global_map_);
  // downsample
  auto filtered_scan = coarse_voxel_filter_->apply(current_lidar_data_.point_cloud);
  auto filtered_map = coarse_voxel_filter_->apply(local_map);
  coarse_registration_->set_target(filtered_map);
  // match
  for (size_t i = 0; i < coarse_matching_yaw_count_; i++) {
    Eigen::Matrix4d predict_pose = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd r_z(2 * M_PI * i / coarse_matching_yaw_count_, Eigen::Vector3d::UnitZ());
    predict_pose.block<3, 1>(0, 3) = coarse_position;
    predict_pose.block<3, 3>(0, 0) = r_z.matrix();
    coarse_registration_->match(filtered_scan, predict_pose);
    if (coarse_registration_->get_fitness_score() < final_error) {
      final_error = coarse_registration_->get_fitness_score();
      initial_pose = coarse_registration_->get_final_pose();
    }
  }
  std::cout << "coarse matching error: " << final_error << std::endl;
  if (final_error > coarse_matching_error_threshold_) {
    return false;
  }
  return true;
}

bool LidarLocalization::get_initial_pose_by_coarse_pose(
  const Eigen::Matrix4d & coarse_pose, Eigen::Matrix4d & initial_pose)
{
  // get local map
  box_filter_->set_origin(coarse_pose.block<3, 1>(0, 3));
  auto local_map = box_filter_->apply(global_map_);
  // downsample
  auto filtered_scan = coarse_voxel_filter_->apply(current_lidar_data_.point_cloud);
  auto filtered_map = coarse_voxel_filter_->apply(local_map);
  coarse_registration_->set_target(filtered_map);
  // match
  coarse_registration_->match(filtered_scan, coarse_pose);
  auto final_error = coarse_registration_->get_fitness_score();
  std::cout << "coarse matching error: " << final_error << std::endl;
  if (final_error > coarse_matching_error_threshold_) {
    return false;
  }
  initial_pose = coarse_registration_->get_final_pose();
  return true;
}

bool LidarLocalization::get_initial_pose_by_scan_context(Eigen::Matrix4d & initial_pose)
{
  // place recognition by using scan context
  if (!scan_context_manager_->detect_loop_closure(current_lidar_data_.point_cloud)) {
    return false;
  }
  Eigen::Matrix4d proposal_pose = scan_context_manager_->get_pose();
  // coarse matching by using position, because yaw in scan context is inaccurate.
  if (!get_initial_pose_by_coarse_position(proposal_pose.block<3, 1>(0, 3), initial_pose)) {
    return false;
  }
  std::cout << "successed to get initial pose by scan context." << std::endl;
  return true;
}

bool LidarLocalization::get_initial_pose_by_gnss_data(Eigen::Matrix4d & initial_pose)
{
  // find nearest gnss data
  double min_dt = gnss_data_time_threshold_;
  Eigen::Vector3d gnss_position;
  for (size_t i = 0; i < gnss_data_buffer_.size(); i++) {
    double dt = fabs(gnss_data_buffer_[i].time - current_lidar_data_.time);
    if (dt < min_dt) {
      gnss_position = gnss_data_buffer_[i].antenna_position;
      min_dt = dt;
    }
  }
  std::cout << "get the nearest gnss data, time diffence: " << min_dt << std::endl;
  if (min_dt >= gnss_data_time_threshold_) {
    return false;
  }
  // coarse matching by raw gnss position
  if (!get_initial_pose_by_coarse_position(gnss_position, initial_pose)) {
    return false;
  }
  std::cout << "successed to get initial pose by gnss data(antenna_position)." << std::endl;
  return true;
}

bool LidarLocalization::get_initial_pose_by_gnss_odometry(Eigen::Matrix4d & initial_pose)
{
  if (gnss_odom_buffer_->size() == 0) {
    return false;
  }
  // get gnss odom at time of current_lidar_data
  localization_common::OdomData odom;
  if (!gnss_odom_buffer_->get_interpolated_data(current_lidar_data_.time, odom)) {
    // get the nearest gnss odometry if can't interpolate odom
    gnss_odom_buffer_->get_nearest_data(current_lidar_data_.time, odom);
  }
  double min_dt = fabs(odom.time - current_lidar_data_.time);
  std::cout << "the time diffence of gnss odometry: " << min_dt << std::endl;
  if (min_dt >= gnss_odometry_time_threshold_) {
    return false;
  }
  Eigen::Matrix4d coarse_pose = odom.pose * T_base_lidar_;
  // coarse matching
  if (!get_initial_pose_by_coarse_pose(coarse_pose, initial_pose)) {
    return false;
  }
  std::cout << "successed to get initial pose by gnss odometry." << std::endl;
  return true;
}

bool LidarLocalization::init_global_localization()
{
  Eigen::Matrix4d initial_pose;
  // 1. try to get initial pose by scan context
  bool success = false;
  if (!success && use_scan_context_) {
    success = get_initial_pose_by_scan_context(initial_pose);
  }
  // 2. try to get initial pose by gnss odometry
  if (!success && use_gnss_odometry_) {
    success = get_initial_pose_by_gnss_odometry(initial_pose);
  }
  // 3. try to get initial pose by gnss data
  if (!success && use_gnss_data_) {
    success = get_initial_pose_by_gnss_data(initial_pose);
  }
  if (!success) {
    std::cout << "failed to get initial pose!" << std::endl;
    return false;
  }
  // reset local map
  update_local_map(initial_pose.block<3, 1>(0, 3));
  // match lidar data
  match_scan_to_map(initial_pose);
  // debug info
  std::cout << "successed to initialize global localization." << std::endl
            << " initial position: " << initial_pose.block<3, 1>(0, 3).transpose() << std::endl
            << " final position  : " << current_lidar_frame_.pose.block<3, 1>(0, 3).transpose()
            << std::endl;
  return true;
}

}  // namespace lidar_localization
