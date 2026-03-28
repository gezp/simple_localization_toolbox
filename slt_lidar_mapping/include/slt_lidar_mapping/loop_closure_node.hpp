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

#include <memory>
#include <vector>
//
#include "rclcpp/rclcpp.hpp"
#include "slt_interface/srv/save_scan_context.hpp"
#include "slt_common/subscriber/cloud_subscriber.hpp"
#include "slt_common/subscriber/lidar_frames_subscriber.hpp"
#include "slt_common/publisher/loop_candidate_publisher.hpp"
#include "slt_lidar_mapping/loop_closure.hpp"

namespace slt_lidar_mapping
{
class LoopClosureNode
{
public:
  explicit LoopClosureNode(rclcpp::Node::SharedPtr node);
  ~LoopClosureNode();

private:
  bool run();

private:
  rclcpp::Node::SharedPtr node_;
  // sub & pub
  std::shared_ptr<slt_common::LidarFramesSubscriber> key_frames_sub_;
  std::shared_ptr<slt_common::LoopCandidatePublisher> loop_candidate_pub_;
  // srv
  rclcpp::Service<slt_interface::srv::SaveScanContext>::SharedPtr save_scan_context_srv_;
  bool save_scan_context_flag_{false};
  // loop closing and flow thread
  std::shared_ptr<LoopClosure> loop_closure_;
  std::unique_ptr<std::thread> run_thread_;
  bool exit_{false};
  // data
  std::vector<slt_common::LidarFrame> key_frames_;
  size_t last_key_frame_count_ = 0;
};
}  // namespace slt_lidar_mapping
