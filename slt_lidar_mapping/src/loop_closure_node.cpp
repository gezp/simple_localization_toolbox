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

#include "slt_lidar_mapping/loop_closure_node.hpp"

#include <filesystem>

namespace slt_lidar_mapping
{
LoopClosureNode::LoopClosureNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  std::string loop_closure_config;
  std::string data_path;
  node->declare_parameter("loop_closure_config", loop_closure_config);
  node->declare_parameter("data_path", data_path);
  node->get_parameter("loop_closure_config", loop_closure_config);
  node->get_parameter("data_path", data_path);
  RCLCPP_INFO(node->get_logger(), "loop_closure_config: [%s]", loop_closure_config.c_str());
  RCLCPP_INFO(node->get_logger(), "data_path: [%s]", data_path.c_str());
  if (loop_closure_config == "" || (!std::filesystem::exists(loop_closure_config))) {
    RCLCPP_FATAL(node->get_logger(), "loop_closure_config is invalid");
    return;
  }
  if (data_path == "" || (!std::filesystem::is_directory(data_path))) {
    RCLCPP_FATAL(node->get_logger(), "data_path is invalid");
    return;
  }
  // loop closing
  loop_closure_ = std::make_shared<LoopClosure>();
  std::cout << "-----------------Init Loop-Closing Detection-------------------" << std::endl;
  loop_closure_->init_config(loop_closure_config, data_path);
  // subscriber:
  key_frames_sub_ =
    std::make_shared<slt_common::LidarFramesSubscriber>(node, "key_frames", 100);
  // publisher
  loop_candidate_pub_ =
    std::make_shared<slt_common::LoopCandidatePublisher>(node, "loop_candidate", 100);
  // save map callback
  save_scan_context_srv_ = node->create_service<slt_interface::srv::SaveScanContext>(
    "save_scan_context",
    [this](
      const slt_interface::srv::SaveScanContext::Request::SharedPtr /*request*/,
      slt_interface::srv::SaveScanContext::Response::SharedPtr response) {
      save_scan_context_flag_ = true;
      response->succeed = true;
    });
  // process loop callback
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        run();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
      }
    });
}

LoopClosureNode::~LoopClosureNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool LoopClosureNode::run()
{
  if (save_scan_context_flag_) {
    // save scan_context
    loop_closure_->save();
    save_scan_context_flag_ = false;
  }
  key_frames_sub_->parse_data(key_frames_);
  if (key_frames_.size() == 0) {
    return false;
  }
  if (key_frames_.size() < last_key_frame_count_) {
    RCLCPP_ERROR(node_->get_logger(), "invalid key frames!");
    return false;
  }
  loop_closure_->reset(key_frames_);
  for (size_t i = last_key_frame_count_; i < key_frames_.size(); i++) {
    if (loop_closure_->detect(key_frames_[i])) {
      loop_candidate_pub_->publish(loop_closure_->get_loop_candidate());
    }
  }
  last_key_frame_count_ = key_frames_.size();
  key_frames_.clear();
  return true;
}

}  // namespace slt_lidar_mapping
