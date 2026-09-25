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

#include "slt_lio/lio_node.hpp"

#include <filesystem>

#include "slt_lio/fastlio.hpp"
#include "slt_lio/fastlio2.hpp"

namespace slt_lio
{

LioNode::LioNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  std::string lio_config;
  node->declare_parameter("lio_config", lio_config);
  node->declare_parameter("publish_tf", publish_tf_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("imu_frame_id", imu_frame_id_);
  node->declare_parameter("lidar_frame_id", lidar_frame_id_);
  node->declare_parameter("odom_frame_id", odom_frame_id_);
  node->get_parameter("lio_config", lio_config);
  node->get_parameter("publish_tf", publish_tf_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("imu_frame_id", imu_frame_id_);
  node->get_parameter("lidar_frame_id", lidar_frame_id_);
  node->get_parameter("odom_frame_id", odom_frame_id_);
  RCLCPP_INFO(node->get_logger(), "lio_config: [%s]", lio_config.c_str());
  if (lio_config == "" || (!std::filesystem::exists(lio_config))) {
    RCLCPP_FATAL(node->get_logger(), "lio_config is invalid");
    return;
  }
  // the algorithm module is self contained, it owns its filter & initializer
  YAML::Node config = YAML::LoadFile(lio_config);
  std::string method = config["lio_method"].as<std::string>();
  if (method == "fastlio") {
    lio_ = std::make_shared<Fastlio>(config["fastlio"]);
  } else if (method == "fastlio2") {
    lio_ = std::make_shared<Fastlio2>(config["fastlio2"]);
  } else {
    RCLCPP_FATAL(node->get_logger(), "unknown lio method: %s\n", method.c_str());
    return;
  }
  bool enable = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enable);
  elapsed_time_statistics_.set_title("LioNode");
  cloud_sub_ = std::make_shared<slt_common::CloudSubscriber>(node, "synced_cloud", 10000);
  imu_sub_ = std::make_shared<slt_common::ImuSubscriber>(node, "imu", 100000);
  current_scan_pub_ = std::make_shared<slt_common::CloudPublisher>(
    node, "lidar_odometry/current_scan", odom_frame_id_, 100);
  local_map_pub_ = std::make_shared<slt_common::CloudPublisher>(
    node, "lidar_odometry/local_map", odom_frame_id_, 100);
  lidar_odom_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "lidar_odometry/odom", odom_frame_id_, base_frame_id_, 100);
  imu_odom_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "lidar_odometry/imu_odom", odom_frame_id_, base_frame_id_, 100);
  if (publish_tf_) {
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    // the continuous stream owns the transform, it does not wait for an observation
    imu_odom_pub_->set_tf_broadcaster(tf_pub_);
  }
  extrinsics_manager_ = std::make_shared<slt_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  // the poll has to stay fine: run() feeds the filter the imu imu_thread_ collected
  // since the last poll, so a coarse poll hands it coarse batches (a batch spanning the
  // initialization loses the samples the initializer consumes)
  run_thread_ = std::make_unique<std::thread>([this]() {
    while (!exit_) {
      if (!run()) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(20ms);
      }
    }
  });
  // the imu has its own thread: it owns the subscription, so both the odometry and the
  // filter's imu feed stay at the imu rate however long a scan takes
  imu_thread_ = std::make_unique<std::thread>([this]() {
    while (!exit_) {
      if (!update_imu_odom()) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(2ms);
      }
    }
  });
}

LioNode::~LioNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
  if (imu_thread_) {
    imu_thread_->join();
  }
}

bool LioNode::run()
{
  // update extrinsics
  if (!is_valid_extrinsics_) {
    if (
      !extrinsics_manager_->lookup(imu_frame_id_, lidar_frame_id_, T_imu_lidar_) ||
      !extrinsics_manager_->lookup(base_frame_id_, imu_frame_id_, T_base_imu_)) {
      return false;
    }
    lio_->set_extrinsic(T_imu_lidar_, T_base_imu_);
    is_valid_extrinsics_ = true;
  }
  // read data
  cloud_sub_->parse_data(lidar_data_buffer_);
  if (lidar_data_buffer_.empty()) {
    return false;
  }
  // the imu is parsed by imu_thread_, which queues the filter's share for us
  {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    while (!imu_data_buffer_.empty()) {
      lio_->add_imu_data(imu_data_buffer_.front());
      imu_data_buffer_.pop_front();
    }
  }
  while (!lidar_data_buffer_.empty()) {
    lio_->add_lidar_data(lidar_data_buffer_.front());
    lidar_data_buffer_.pop_front();
  }
  elapsed_time_statistics_.tic("update lio");
  // the filter consumes the observations and leaves its corrected state
  if (lio_->update()) {
    slt_common::OdomData corrected_odom;
    {
      std::lock_guard<std::mutex> lock(corrected_state_mutex_);
      corrected_state_ = lio_->get_imu_nav_state();
      corrected_imu_ = lio_->get_imu_data();
      corrected_odom = to_odom(corrected_state_, corrected_imu_);
      // the odometry must be re-seeded from it, the imu thread clears this
      has_new_corrected_state_ = true;
    }
    lidar_odom_pub_->publish(corrected_odom);
    // display for debug
    if (current_scan_pub_->has_subscribers()) {
      auto current_scan = lio_->get_current_scan();
      if (current_scan != nullptr) {
        current_scan_pub_->publish(*current_scan);
      }
    }
    if (local_map_pub_->has_subscribers()) {
      auto local_map = lio_->get_local_map();
      if (local_map != nullptr) {
        local_map_pub_->publish(*local_map);
      }
    }
  }
  elapsed_time_statistics_.toc("update lio");
  elapsed_time_statistics_.print_all_info("update lio", 100);
  return true;
}

bool LioNode::update_imu_odom()
{
  std::deque<slt_common::ImuData> new_imus;
  imu_sub_->parse_data(new_imus);
  {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_buffer_.insert(imu_data_buffer_.end(), new_imus.begin(), new_imus.end());
  }
  // reset imu_odom when an observation lands: the only cross thread part of this
  // function, the rest is this thread's own
  {
    std::lock_guard<std::mutex> lock(corrected_state_mutex_);
    if (has_new_corrected_state_) {
      has_new_corrected_state_ = false;
      imu_odom_.reset(corrected_state_, true);
      imu_odom_.integrate(corrected_imu_);
      is_initialized_ = true;
    }
  }
  for (const auto & imu_data : new_imus) {
    imu_history_buffer_.emplace(imu_data.time, imu_data);
    while (imu_history_buffer_.size() > kMaxImuBuffer) {
      imu_history_buffer_.erase(imu_history_buffer_.begin());
    }
  }
  if (!is_initialized_ || new_imus.empty()) {
    return false;
  }
  const double odom_time = imu_odom_.get_imu_nav_state().time;
  for (auto it = imu_history_buffer_.upper_bound(odom_time); it != imu_history_buffer_.end();
       ++it) {
    imu_odom_.integrate(it->second);
  }
  // a correction only goes out once the replay carried the odometry past the stamp
  // already published, so the stream stays at imu rate and monotonic
  const double new_odom_time = imu_odom_.get_imu_nav_state().time;
  if (new_odom_time <= published_imu_odom_time_) {
    return false;
  }
  published_imu_odom_time_ = new_odom_time;
  imu_odom_pub_->publish(to_odom(imu_odom_.get_imu_nav_state(), imu_odom_.get_imu_data()));
  return true;
}

slt_common::OdomData LioNode::to_odom(
  const slt_common::ImuNavState & nav_state, const slt_common::ImuData & imu_data)
{
  slt_common::OdomData odom;
  odom.time = nav_state.time;
  odom.pose.block<3, 1>(0, 3) = nav_state.position;
  odom.pose.block<3, 3>(0, 0) = nav_state.orientation;
  odom.linear_velocity = nav_state.linear_velocity;
  odom.angular_velocity = imu_data.angular_velocity;
  return slt_common::transform_odom(odom, T_base_imu_);
}

}  // namespace slt_lio
