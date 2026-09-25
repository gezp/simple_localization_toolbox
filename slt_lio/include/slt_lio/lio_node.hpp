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

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "slt_common/extrinsics_manager.hpp"
#include "slt_common/publisher/cloud_publisher.hpp"
#include "slt_common/publisher/odometry_publisher.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/sensor_data_utils.hpp"
#include "slt_common/subscriber/cloud_subscriber.hpp"
#include "slt_common/subscriber/imu_subscriber.hpp"
#include "slt_common/tic_toc.hpp"
#include "slt_imu_odometry/imu_integration.hpp"
#include "slt_lio/lio_interface.hpp"

namespace slt_lio
{

// node shell for the lidar-inertial odometry: feeds imu/lidar into the selected
// algorithm module (fastlio/fastlio2, self contained) and drives the high frequency
// odometry the filter does not own, re-seeding it from the corrected state.
// two threads: run() feeds the filter & publishes the display data, imu_thread_ owns
// the imu subscription and integrates/publishes the odometry. they meet on
// imu_data_mutex_ and corrected_state_mutex_ (never held together), which the filter
// never touches.
class LioNode
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  explicit LioNode(rclcpp::Node::SharedPtr node);
  ~LioNode();

private:
  bool run();
  // imu_thread_ body: parse the imu, integrate & publish the odometry; true when published
  bool update_imu_odom();
  // odometry of the base frame at `nav_state`, `imu_data` supplies the twist
  slt_common::OdomData to_odom(
    const slt_common::ImuNavState & nav_state, const slt_common::ImuData & imu_data);

private:
  rclcpp::Node::SharedPtr node_;
  // pub & sub
  std::shared_ptr<slt_common::CloudSubscriber> cloud_sub_;
  std::shared_ptr<slt_common::ImuSubscriber> imu_sub_;
  std::shared_ptr<slt_common::CloudPublisher> current_scan_pub_;
  std::shared_ptr<slt_common::CloudPublisher> local_map_pub_;
  // the corrected state, one message per consumed observation
  std::shared_ptr<slt_common::OdometryPublisher> lidar_odom_pub_;
  // the imu driven odometry, re-seeded from the corrected state at every observation
  std::shared_ptr<slt_common::OdometryPublisher> imu_odom_pub_;
  // tf
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<slt_common::ExtrinsicsManager> extrinsics_manager_;
  std::string lidar_frame_id_{"lidar"};
  std::string imu_frame_id_{"imu"};
  std::string base_frame_id_{"base"};
  std::string odom_frame_id_{"map"};
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_imu_ = Eigen::Matrix4d::Identity();
  bool is_valid_extrinsics_{false};
  bool publish_tf_{false};
  // algorithm module (fastlio/fastlio2, selected by lio_method)
  std::shared_ptr<LioInterface> lio_;
  std::unique_ptr<std::thread> run_thread_;
  std::unique_ptr<std::thread> imu_thread_;
  std::atomic<bool> exit_{false};
  // filled from the subscriber queues (run() has the lidar, imu_thread_ the imu)
  std::deque<slt_common::LidarData> lidar_data_buffer_;
  // the filter's share of the imu batch, imu_thread_ -> run()
  std::mutex imu_data_mutex_;
  std::deque<slt_common::ImuData> imu_data_buffer_;
  // ---- the corrected state is all that crosses back, guarded by
  // corrected_state_mutex_. everything below is imu_thread_'s own ----
  std::mutex corrected_state_mutex_;
  // the state of the last observation and the imu sample it sits on (the filter's own):
  // the odometry is seeded with them, the first replayed step starts there
  slt_common::ImuNavState corrected_state_;
  slt_common::ImuData corrected_imu_;
  // has_new_corrected_state_: not consumed yet, the odometry must be re-seeded from it
  bool has_new_corrected_state_{false};
  // is_initialized_: the odometry has been seeded, corrected_state_ holds a value
  bool is_initialized_{false};
  slt_imu_odometry::ImuIntegration imu_odom_;
  // imu history by time: the replay backlog the odometry is carried forward with after
  // a correction. bounded by kMaxImuBuffer, oldest first
  std::map<double, slt_common::ImuData> imu_history_buffer_;
  // the imu odom stamp already published: a correction goes out once the replay passed it
  double published_imu_odom_time_{0.0};
  static constexpr size_t kMaxImuBuffer = 500;
  // debug
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};

}  // namespace slt_lio
