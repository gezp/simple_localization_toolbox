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

#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <deque>
//
#include "slt_common/cloud_filter/box_filter.hpp"
#include "slt_common/cloud_filter/voxel_filter.hpp"
#include "slt_common/cloud_registration/cloud_registration_factory.hpp"
#include "slt_common/cloud_registration/cloud_registration_interface.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/sensor_data/lidar_frame.hpp"
#include "slt_common/sensor_data/gnss_data.hpp"
#include "slt_common/sensor_data/odom_data.hpp"
#include "slt_common/odom_data_buffer.hpp"
#include "scan_context/scan_context_manager.hpp"

namespace slt_lidar_locator
{
class LidarLocalization
{
public:
  LidarLocalization();
  ~LidarLocalization() = default;
  bool init_config(const std::string & config_path, const std::string & data_path);
  void set_extrinsic(const Eigen::Matrix4d & T_base_lidar);
  bool add_gnss_data(const slt_common::GnssData & gnss_data);
  bool add_gnss_odom(const slt_common::OdomData & gnss_odom);
  bool update(const slt_common::LidarData<slt_common::PointXYZIRT> & lidar_data);
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_current_scan();
  slt_common::OdomData get_current_odom();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_global_map();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map();
  bool has_new_local_map();

private:
  bool init_global_localization_config(const YAML::Node & config_node);
  bool check_new_local_map(const Eigen::Matrix4d & pose);
  bool update_local_map(const Eigen::Vector3d & position);
  bool match_scan_to_map(const Eigen::Matrix4d & predict_pose);
  // initial pose
  bool get_initial_pose_by_history(Eigen::Matrix4d & initial_pose);
  bool get_initial_pose_by_coarse_position(
    const Eigen::Vector3d & coarse_position, Eigen::Matrix4d & initial_pose);
  bool get_initial_pose_by_coarse_pose(
    const Eigen::Matrix4d & coarse_pose, Eigen::Matrix4d & initial_pose);
  bool get_initial_pose_by_scan_context(Eigen::Matrix4d & initial_pose);
  bool get_initial_pose_by_gnss_data(Eigen::Matrix4d & initial_pose);
  bool get_initial_pose_by_gnss_odometry(Eigen::Matrix4d & initial_pose);
  bool init_global_localization();

private:
  std::shared_ptr<scan_context::ScanContextManager> scan_context_manager_;
  std::shared_ptr<slt_common::CloudRegistrationFactory> registration_factory_;
  std::shared_ptr<slt_common::CloudRegistrationInterface> registration_;
  std::shared_ptr<slt_common::CloudRegistrationInterface> coarse_registration_;
  std::shared_ptr<slt_common::BoxFilter> box_filter_;
  std::shared_ptr<slt_common::VoxelFilter> local_map_filter_;
  std::shared_ptr<slt_common::VoxelFilter> current_scan_filter_;
  std::shared_ptr<slt_common::VoxelFilter> display_filter_;
  std::shared_ptr<slt_common::VoxelFilter> coarse_voxel_filter_;
  // data path
  std::string data_path_ = "";
  std::string scan_context_path_ = "";
  std::string map_path_ = "";
  // params for global localization
  bool use_scan_context_{false};
  bool use_gnss_odometry_{false};
  bool use_gnss_data_{false};
  size_t coarse_matching_yaw_count_{12};
  double coarse_matching_error_threshold_{1.0};
  double gnss_odometry_time_threshold_{0.5};
  double gnss_data_time_threshold_{0.5};
  // tf
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_base_ = Eigen::Matrix4d::Identity();
  // data
  bool has_inited_ = false;
  bool has_new_local_map_ = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_;
  slt_common::LidarData<pcl::PointXYZ> current_lidar_data_;
  slt_common::LidarFrame current_lidar_frame_;
  std::deque<slt_common::LidarFrame> history_frames_;
  std::deque<slt_common::GnssData> gnss_data_buffer_;
  std::shared_ptr<slt_common::OdomDataBuffer> gnss_odom_buffer_;
};
}  // namespace slt_lidar_locator
