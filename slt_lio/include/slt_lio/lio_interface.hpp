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

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"

namespace slt_lio
{

// interface of a tightly coupled lidar-inertial odometry: it owns the filter, the
// high frequency odometry is the caller's (the corrected state re-seeds it)
class LioInterface
{
public:
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

  virtual ~LioInterface() {}
  // T_imu_lidar: lidar frame -> imu frame, T_base_imu: imu frame -> base frame
  // (the odometry output frame)
  virtual void set_extrinsic(
    const Eigen::Matrix4d & T_imu_lidar, const Eigen::Matrix4d & T_base_imu) = 0;
  // imu tick: seeds the initializer before init, propagation input after
  virtual void add_imu_data(const slt_common::ImuData & imu_data) = 0;
  // scan in lidar frame: buffered and registered by update()
  virtual void add_lidar_data(const slt_common::LidarData & lidar_data) = 0;
  // initialize if needed, then consume every observation the buffered imu already
  // covers, oldest first; true when an observation was consumed
  virtual bool update() = 0;
  virtual bool is_initialized() const = 0;
  // state corrected by the last consumed observation, defined at its time stamp
  virtual slt_common::ImuNavState get_imu_nav_state() = 0;
  // the imu sample of get_imu_nav_state().time, the one the state was propagated with:
  // it carries the twist the state was observed at. defined once update() consumed one
  virtual slt_common::ImuData get_imu_data() = 0;
  // for display, current point cloud & local map in odometry frame, nullptr when they
  // did not change since the last call (an implementation without display: the default)
  virtual PointCloudPtr get_current_scan() {return nullptr;}
  virtual PointCloudPtr get_local_map() {return nullptr;}
};

}  // namespace slt_lio
