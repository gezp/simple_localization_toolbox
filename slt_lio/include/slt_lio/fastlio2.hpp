// Copyright 2026 Zhenpeng Ge (https://github.com/zhenpengge).
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
#include <ikd_tree/ikd_Tree.h>

#include <deque>
#include <map>
#include <memory>

#include "slt_common/point_cloud_filter/voxel_filter.hpp"
#include "slt_common/point_cloud_registration/point_cloud_registration_interface.hpp"
#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/sensor_data_utils.hpp"
#include "slt_common/tic_toc.hpp"
#include "slt_lio/kalman_filter/eskf.hpp"
#include "slt_lio/lio_initializer.hpp"
#include "slt_lio/lio_interface.hpp"

namespace slt_lio
{

// FAST-LIO2 style tightly coupled lidar-inertial odometry: owns the eskf, the imu/lidar
// buffers and the initialization, registers against an incremental ikd-tree local map
// (add voxelized scan points, crop by box around the current position).
// reference: FAST-LIO2 (https://github.com/hku-mars/FAST_LIO)
// the filter stops at the last consumed observation, so each observation is registered
// at its own time stamp; the high frequency odometry is the caller's, re-seeded from
// get_imu_nav_state()
class Fastlio2 : public LioInterface
{
  using IkdTree = KD_TREE<pcl::PointXYZ>;

public:
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

  explicit Fastlio2(const YAML::Node & config);
  ~Fastlio2() override;
  void set_extrinsic(
    const Eigen::Matrix4d & T_imu_lidar, const Eigen::Matrix4d & T_base_imu) override;
  void add_imu_data(const slt_common::ImuData & imu_data) override;
  void add_lidar_data(const slt_common::LidarData & lidar_data) override;
  bool update() override;
  bool is_initialized() const override;
  slt_common::ImuNavState get_imu_nav_state() override;
  slt_common::ImuData get_imu_data() override;
  PointCloudPtr get_current_scan() override;
  PointCloudPtr get_local_map() override;

private:
  // initialize the eskf once the initializer gets an initial state
  bool try_init();
  bool observe_point_cloud();
  // current imu pose in world frame
  Eigen::Matrix4d get_current_pose();
  bool update_local_map();
  // advance the eskf to the observation time, then register the scan
  void consume_observation(const slt_common::LidarData & lidar_data, double t_obs);
  // predict the eskf with the buffered imu newer than its state, until it reaches time
  void propagate_eskf_to(double time);
  // the imu sample of `time`: interpolated from the neighbours bracketing it, or the
  // nearest one when `time` falls outside the window (the history must not be empty)
  slt_common::ImuData sample_imu(double time);

private:
  // the history is kept across update() calls, so an observation found late still has
  // the samples it needs. the scan is a sweep whose cloud carries no per-point time to
  // deskew with: its points describe the end of the sweep while the stamp is its start
  static constexpr double kScanDuration = 0.1;
  static constexpr size_t kMaxImuBuffer = 500;
  static constexpr size_t kMaxLidarBuffer = 50;
  YAML::Node config_;
  // integrates the imu, corrected by the point-to-plane observation
  std::unique_ptr<Eskf> eskf_;
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R_imu_lidar_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_imu_lidar_ = Eigen::Vector3d::Zero();
  bool has_extrinsic_{false};
  bool is_inited_{false};
  std::shared_ptr<slt_common::VoxelFilter> input_filter_;
  std::shared_ptr<slt_common::VoxelFilter> display_filter_;
  std::deque<slt_common::LidarData> lidar_buffer_;
  // imu history by time, kept across update() calls: what the eskf is retimed with at
  // an observation. bounded by kMaxImuBuffer, oldest first
  std::map<double, slt_common::ImuData> imu_history_buffer_;
  // last registered scan in imu frame at the observation time
  PointCloudPtr current_scan_;
  std::unique_ptr<LioInitializer> initializer_;
  int num_nearby_{5};
  double nearby_distance_{1.0};
  double plane_eigen_ratio_{3.0};
  double max_plane_distance_{0.2};
  int max_num_iterations_{4};
  std::unique_ptr<IkdTree> ikd_tree_;
  double map_crop_range_{100.0};
  // sliding local-map cube (crop), see update_local_map
  BoxPointType local_map_points_{};
  bool local_map_initialized_{false};
  bool has_new_scan_{false};
  bool has_new_local_map_{false};
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};

}  // namespace slt_lio
