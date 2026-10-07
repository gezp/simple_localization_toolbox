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

#include "slt_vio/vio_node.hpp"

#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>

#include "slt_common/sensor_data/image_data.hpp"
#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/sensor_data_utils.hpp"

namespace slt_vio
{
VioNode::VioNode(rclcpp::Node::SharedPtr node)
: node_(node)
{
  std::string vio_config;
  node->declare_parameter("vio_config", vio_config);
  node->declare_parameter("enable_compressed", enable_compressed_);
  node->declare_parameter("camera_frame_id", camera_frame_id_);
  node->declare_parameter("odom_frame_id", odom_frame_id_);
  node->declare_parameter("base_frame_id", base_frame_id_);
  node->declare_parameter("imu_frame_id", imu_frame_id_);
  node->declare_parameter("publish_tf", publish_tf_);
  node->get_parameter("vio_config", vio_config);
  node->get_parameter("enable_compressed", enable_compressed_);
  node->get_parameter("camera_frame_id", camera_frame_id_);
  node->get_parameter("odom_frame_id", odom_frame_id_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("imu_frame_id", imu_frame_id_);
  node->get_parameter("publish_tf", publish_tf_);
  RCLCPP_INFO(node->get_logger(), "vio_config: [%s]", vio_config.c_str());
  if (vio_config == "" || (!std::filesystem::exists(vio_config))) {
    RCLCPP_FATAL(node->get_logger(), "vio_config is invalid");
    return;
  }
  const YAML::Node config = YAML::LoadFile(vio_config);
  vins_mono_ = std::make_unique<VinsMono>(config["vins_mono"]);
  elapsed_time_statistics_.set_enable(
    config["enable_elapsed_time_statistics"].as<bool>(false));
  elapsed_time_statistics_.set_title("VioNode");

  image_sub_ =
    std::make_shared<slt_common::ImageSubscriber>(node, "image", 100, enable_compressed_);
  camera_info_sub_ = std::make_shared<slt_common::CameraInfoSubscriber>(node, "camera_info");
  imu_sub_ = std::make_shared<slt_common::ImuSubscriber>(node, "imu", 200000);
  feature_image_pub_ = std::make_shared<slt_common::ImagePublisher>(
    node, "visual_odometry/feature_image", camera_frame_id_, 10);
  odom_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "visual_odometry/odom", odom_frame_id_, base_frame_id_, 100);
  imu_odom_pub_ = std::make_shared<slt_common::OdometryPublisher>(
    node, "visual_odometry/imu_odom", odom_frame_id_, base_frame_id_, 100);
  if (publish_tf_) {
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    // the continuous stream owns the transform; it does not wait for an output frame
    imu_odom_pub_->set_tf_broadcaster(tf_pub_);
  }
  extrinsics_manager_ = std::make_shared<slt_common::ExtrinsicsManager>(node);
  extrinsics_manager_->enable_tf_listener();
  run_thread_ = std::make_unique<std::thread>([this]() {
        while (!exit_) {
          if (!this->run()) {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(20ms);
          }
        }
  });
  // imu has its own thread: odometry stays at the imu rate however long image processing takes
  imu_thread_ = std::make_unique<std::thread>([this]() {
        while (!exit_) {
          if (!this->update_imu_odom()) {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(2ms);
          }
        }
  });
}

VioNode::~VioNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
  if (imu_thread_) {
    imu_thread_->join();
  }
}

bool VioNode::run()
{
  if (!is_valid_extrinsics_) {
    if (!extrinsics_manager_->lookup(base_frame_id_, imu_frame_id_, T_base_imu_) ||
      !extrinsics_manager_->lookup(imu_frame_id_, camera_frame_id_, T_imu_camera_))
    {
      return false;
    }
    vins_mono_->set_extrinsic(T_imu_camera_);
    is_valid_extrinsics_ = true;
  }
  if (!has_camera_intrinsic_) {
    slt_common::CameraIntrinsicParam param;
    if (!camera_info_sub_->get_param(param)) {
      return false;
    }
    if (!vins_mono_->set_camera_intrinsic(param)) {
      return false;
    }
    RCLCPP_INFO(
      node_->get_logger(), "camera intrinsics: fx:%f fy:%f cx:%f cy:%f, %dx%d", param.intrinsics[0],
      param.intrinsics[1], param.intrinsics[2], param.intrinsics[3], param.width, param.height);
    has_camera_intrinsic_ = true;
  }
  std::deque<slt_common::ImageData> image_buffer;
  image_sub_->parse_data(image_buffer);
  if (image_buffer.empty()) {
    return false;
  }
  for (const auto & data : image_buffer) {
    vins_mono_->add_image_data(data);
  }
  // the imu is parsed by imu_thread_, which queues the module's share
  {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    while (!imu_data_buffer_.empty()) {
      vins_mono_->add_imu_data(imu_data_buffer_.front());
      imu_data_buffer_.pop_front();
    }
  }
  elapsed_time_statistics_.tic("update vio");
  // the state is left only once the window is initialized; earlier frames only feed the module
  if (vins_mono_->update()) {
    slt_common::OdomData corrected_odom;
    {
      std::lock_guard<std::mutex> lock(corrected_state_mutex_);
      corrected_state_ = vins_mono_->get_imu_nav_state();
      corrected_imu_ = vins_mono_->get_imu_data();
      corrected_odom = to_odom(corrected_state_, corrected_imu_);
      // the odometry must be re-seeded from it; the imu thread clears this
      has_new_corrected_state_ = true;
    }
    odom_pub_->publish(corrected_odom);
    // tracking is drawn only when someone looks, carrying the frame's stamp
    if (feature_image_pub_->has_subscribers()) {
      feature_image_pub_->publish(vins_mono_->get_feature_image(), corrected_odom.time);
    }
  }
  elapsed_time_statistics_.toc("update vio");
  elapsed_time_statistics_.print_all_info("update vio", 100);
  return true;
}

bool VioNode::update_imu_odom()
{
  std::deque<slt_common::ImuData> new_imus;
  imu_sub_->parse_data(new_imus);
  {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_buffer_.insert(imu_data_buffer_.end(), new_imus.begin(), new_imus.end());
    // the queue is the module's share waiting for an image: a camera that never arrives must not
    // grow it without bound, the oldest samples go first
    while (imu_data_buffer_.size() > kMaxImuBufferSize) {
      imu_data_buffer_.pop_front();
    }
  }
  // reset imu_odom when a frame lands: the only cross-thread part of this function
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
    while (imu_history_buffer_.size() > kMaxImuBufferSize) {
      imu_history_buffer_.erase(imu_history_buffer_.begin());
    }
  }
  if (!is_initialized_ || new_imus.empty()) {
    return false;
  }
  const double odom_time = imu_odom_.get_imu_nav_state().time;
  for (auto it = imu_history_buffer_.upper_bound(odom_time); it != imu_history_buffer_.end();
    ++it)
  {
    imu_odom_.integrate(it->second);
  }
  // correction only once the replay passed the published stamp, keeping odometry monotonic
  const double new_odom_time = imu_odom_.get_imu_nav_state().time;
  if (new_odom_time <= published_imu_odom_time_) {
    return false;
  }
  published_imu_odom_time_ = new_odom_time;
  imu_odom_pub_->publish(to_odom(imu_odom_.get_imu_nav_state(), imu_odom_.get_imu_data()));
  return true;
}

slt_common::OdomData VioNode::to_odom(
  const slt_common::ImuNavState & nav_state, const slt_common::ImuData & imu_data) const
{
  slt_common::OdomData odom;
  odom.time = nav_state.time;
  odom.pose.block<3, 1>(0, 3) = nav_state.position;
  odom.pose.block<3, 3>(0, 0) = nav_state.orientation;
  odom.linear_velocity = nav_state.linear_velocity;
  odom.angular_velocity = imu_data.angular_velocity;
  return slt_common::transform_odom(odom, T_base_imu_);
}

}  // namespace slt_vio
