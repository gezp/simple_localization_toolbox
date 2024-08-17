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

#include "lidar_odometry/loam_odometry.hpp"

#include "localization_common/sensor_data_utils.hpp"

namespace lidar_odometry
{

LoamOdometry::LoamOdometry(const YAML::Node & config)
{
  // init registration and filter
  feature_extraction_ =
    std::make_shared<localization_common::LoamFeatureExtraction>(config["loam_feature_extraction"]);
  registration_ =
    std::make_shared<localization_common::LoamRegistration>(config["loam_registration"]);
  using VoxelFilter = localization_common::VoxelFilter;
  display_filter_ = std::make_shared<VoxelFilter>(config["display_filter"]);
  bool enabel = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enabel);
  elapsed_time_statistics_.set_title("LoamOdometry");
  // print info
  std::cout << "display filter:" << std::endl;
  display_filter_->print_info();
}

void LoamOdometry::set_extrinsic(const Eigen::Matrix4d & T_base_lidar)
{
  T_base_lidar_ = T_base_lidar;
  T_lidar_base_ = T_base_lidar.inverse();
}

bool LoamOdometry::update(
  const localization_common::LidarData<localization_common::PointXYZIRT> & lidar_data)
{
  elapsed_time_statistics_.tic("update");
  current_frame_.time = lidar_data.time;
  current_frame_.point_cloud = lidar_data.point_cloud;
  Eigen::Matrix4d final_pose;
  if (history_poses_.empty()) {
    // initialize the first frame
    feature_extraction_->extract(lidar_data.point_cloud, current_frame_.feature);
    final_pose = T_base_lidar_;
  } else {
    // scan to scan matching
    Eigen::Matrix4d predict_pose = Eigen::Matrix4d::Identity();
    if (!get_initial_pose_by_history(predict_pose)) {
      std::cout << "failed to get predict pose by history" << std::endl;
    }
    match_scan_to_scan(predict_pose, final_pose);
  }
  current_frame_.pose = final_pose;
  // add into history_poses_
  update_history_pose(current_frame_.time, final_pose);
  // set target of registration for next matching
  auto feature = feature_extraction_->transform_feature(current_frame_.feature, final_pose);
  registration_->set_target(feature);
  elapsed_time_statistics_.toc("update");
  elapsed_time_statistics_.print_all_info("update", 20);
  return true;
}

bool LoamOdometry::match_scan_to_scan(
  const Eigen::Matrix4d & predict_pose, Eigen::Matrix4d & final_pose)
{
  // extract loam feature
  elapsed_time_statistics_.tic("match_scan_to_scan");
  feature_extraction_->extract(current_frame_.point_cloud, current_frame_.feature);
  // match
  registration_->match(current_frame_.feature, predict_pose);
  // result
  final_pose = registration_->get_final_pose();
  elapsed_time_statistics_.toc("match_scan_to_scan");
  return true;
}

localization_common::OdomData LoamOdometry::get_current_odom()
{
  localization_common::OdomData odom;
  odom.time = current_frame_.time;
  odom.pose = current_frame_.pose * T_lidar_base_;
  if (history_poses_.size() >= 2) {
    auto & pose1 = history_poses_[history_poses_.size() - 2];
    auto & pose2 = history_poses_[history_poses_.size() - 1];
    auto twist = estimate_twist_by_pose(pose1, pose2);
    odom.linear_velocity = twist.linear_velocity;
    odom.angular_velocity = twist.angular_velocity;
  }
  return odom;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LoamOdometry::get_current_scan()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*current_frame_.point_cloud, *current_cloud);
  return display_filter_->apply(current_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoamOdometry::get_feature_scan()
{
  return feature_extraction_->get_feature_point_cloud(current_frame_.feature);
}

bool LoamOdometry::update_history_pose(double time, const Eigen::Matrix4d & pose)
{
  localization_common::PoseData pose_data;
  pose_data.time = time;
  pose_data.pose = pose;
  history_poses_.push_back(pose_data);
  if (history_poses_.size() > 100) {
    history_poses_.pop_front();
  }
  return true;
}

bool LoamOdometry::get_initial_pose_by_history(Eigen::Matrix4d & initial_pose)
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

}  // namespace lidar_odometry
