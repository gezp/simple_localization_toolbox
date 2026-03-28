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
#include <deque>
#include <memory>
#include <string>

#include "slt_common/cloud_filter/voxel_filter.hpp"
#include "slt_common/loam/loam_feature_extraction.hpp"
#include "slt_common/loam/loam_registration.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/sensor_data/odom_data.hpp"
#include "slt_common/sensor_data/pose_data.hpp"
#include "slt_common/tic_toc.hpp"

namespace slt_lidar_odometry
{

class LoamOdometry
{
  struct Frame
  {
    double time;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pcl::PointCloud<slt_common::PointXYZIRT>::Ptr point_cloud;
    slt_common::LoamFeature feature;
  };

public:
  explicit LoamOdometry(const YAML::Node & config);
  void set_extrinsic(const Eigen::Matrix4d & T_base_lidar);
  bool update(const slt_common::LidarData<slt_common::PointXYZIRT> & lidar_data);
  slt_common::OdomData get_current_odom();
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_current_scan();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_feature_scan();

private:
  bool update_history_pose(double time, const Eigen::Matrix4d & pose);
  bool get_initial_pose_by_history(Eigen::Matrix4d & initial_pose);
  bool match_scan_to_scan(const Eigen::Matrix4d & predict_pose, Eigen::Matrix4d & final_pose);

private:
  std::shared_ptr<slt_common::VoxelFilter> display_filter_;
  std::shared_ptr<slt_common::LoamFeatureExtraction> feature_extraction_;
  std::shared_ptr<slt_common::LoamRegistration> registration_;
  // tf
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_base_ = Eigen::Matrix4d::Identity();
  // data
  Frame current_frame_;
  std::deque<slt_common::PoseData> history_poses_;
  // debug
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};

}  // namespace slt_lidar_odometry
