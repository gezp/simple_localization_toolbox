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

#include <thread>
#include <memory>
#include <algorithm>
#include <deque>
#include <string>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
//
#include "slt_interface/srv/save_odometry.hpp"
#include "slt_common/subscriber/odometry_subscriber.hpp"
#include "slt_common/odom_data_buffer.hpp"
#include "slt_common/msg_utils.hpp"

namespace slt_common
{
class SimpleEvaluatorNode
{
public:
  explicit SimpleEvaluatorNode(rclcpp::Node::SharedPtr node);
  ~SimpleEvaluatorNode();

private:
  bool run();
  bool publish_map_odom_tf(const std::string & odom_name, const OdomData & odom);
  void save_pose(std::ofstream & ofs, const OdomData & odom);
  bool save_trajectory();
  bool check_unique_element(const std::vector<std::string> & v);

private:
  rclcpp::Node::SharedPtr node_;
  // odom subscribers
  std::vector<std::string> odom_topics_;
  std::vector<std::string> odom_names_;
  std::vector<std::shared_ptr<OdometrySubscriber>> odom_subs_;
  // pub
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  // srv
  rclcpp::Service<slt_interface::srv::SaveOdometry>::SharedPtr save_odometry_srv_;
  bool save_odometry_flag_{false};
  // data
  std::vector<OdomDataBuffer> odom_data_buffers_;
  size_t reference_odom_index_{0};
  std::vector<bool> map_odom_tf_published_;
  std::string map_frame_id_{"map"};
  //
  std::string trajectory_path_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
};
}  // namespace slt_common
