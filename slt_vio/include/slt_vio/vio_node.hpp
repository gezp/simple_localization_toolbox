// Copyright 2026 Zhenpeng Ge (https://github.com/gezp).
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

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "slt_common/extrinsics_manager.hpp"
#include "slt_common/publisher/image_publisher.hpp"
#include "slt_common/publisher/odometry_publisher.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/sensor_data/odom_data.hpp"
#include "slt_common/subscriber/camera_info_subscriber.hpp"
#include "slt_common/subscriber/image_subscriber.hpp"
#include "slt_common/subscriber/imu_subscriber.hpp"
#include "slt_common/tic_toc.hpp"
#include "slt_imu_odometry/imu_integration.hpp"
#include "slt_vio/vins_mono.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace slt_vio
{
// node shell for the monocular visual inertial odometry: owns the topics, feeds camera and imu
// into VinsMono, publishes the pose. run() drives the module; imu_thread_ publishes odometry at
// the imu rate; the two meet on the imu queue and the corrected state.
class VioNode
{
public:
  explicit VioNode(rclcpp::Node::SharedPtr node);
  ~VioNode();

private:
  // true when it produced a new state
  bool run();
  // imu_thread_ body. true when odometry was published
  bool update_imu_odom();
  // state as odometry; the imu sample supplies the angular velocity the state does not carry
  slt_common::OdomData to_odom(
    const slt_common::ImuNavState & nav_state, const slt_common::ImuData & imu_data) const;

private:
  rclcpp::Node::SharedPtr node_;
  bool enable_compressed_{true};
  std::string camera_frame_id_{"camera"};
  std::string odom_frame_id_{"map"};
  std::string base_frame_id_{"base"};
  std::string imu_frame_id_{"imu"};
  bool publish_tf_{true};

  std::shared_ptr<slt_common::ImageSubscriber> image_sub_;
  std::shared_ptr<slt_common::CameraInfoSubscriber> camera_info_sub_;
  std::shared_ptr<slt_common::ImuSubscriber> imu_sub_;
  std::shared_ptr<slt_common::ImagePublisher> feature_image_pub_;
  std::shared_ptr<slt_common::OdometryPublisher> odom_pub_;
  std::shared_ptr<slt_common::OdometryPublisher> imu_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<slt_common::ExtrinsicsManager> extrinsics_manager_;
  // the module hands out the imu pose; the topics want the base one
  Eigen::Matrix4d T_base_imu_{Eigen::Matrix4d::Identity()};
  // camera the imu pose is anchored to, off the same tf tree
  Eigen::Matrix4d T_imu_camera_{Eigen::Matrix4d::Identity()};
  bool is_valid_extrinsics_{false};

  // the algorithm module (vins-mono)
  std::unique_ptr<VinsMono> vins_mono_;
  // the intrinsics arrive once, on camera_info, after the tf tree
  bool has_camera_intrinsic_{false};
  slt_common::AdvancedTicToc elapsed_time_statistics_;

  std::atomic<bool> exit_{false};
  std::unique_ptr<std::thread> run_thread_;
  std::unique_ptr<std::thread> imu_thread_;

  // filled from the subscribers (run() has the camera, imu_thread_ the imu). the queue is bounded,
  // the oldest samples go first once it overflows
  static constexpr size_t kMaxImuBufferSize = 2000;
  std::deque<slt_common::ImuData> imu_data_buffer_;
  std::mutex imu_data_mutex_;
  // the state the last output frame left: written by run(), read and cleared by imu_thread_, to
  // re-seed propagation
  std::mutex corrected_state_mutex_;
  slt_common::ImuNavState corrected_state_;
  // the frame's imu sample; the state alone carries no angular velocity
  slt_common::ImuData corrected_imu_;
  bool has_new_corrected_state_{false};
  // imu_thread_'s own, no lock needed
  slt_imu_odometry::ImuIntegration imu_odom_;
  bool is_initialized_{false};
  // the samples a re-seed reads again: every one past the frame time, the ones the discarded run
  // had consumed included. bounded, the oldest go first
  std::map<double, slt_common::ImuData> imu_history_buffer_;
  // imu odom stamp already published: a correction goes out only once the replay passed it
  double published_imu_odom_time_{0.0};
};
}  // namespace slt_vio
