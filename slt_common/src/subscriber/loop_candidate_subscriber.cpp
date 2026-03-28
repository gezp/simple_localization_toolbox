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

#include "slt_common/subscriber/loop_candidate_subscriber.hpp"

namespace slt_common
{
LoopCandidateSubscriber::LoopCandidateSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<slt_interface::msg::LoopCandidate>(
    topic_name, buffer_size,
    std::bind(&LoopCandidateSubscriber::msg_callback, this, std::placeholders::_1));
}

void LoopCandidateSubscriber::msg_callback(
  const slt_interface::msg::LoopCandidate::SharedPtr msg)
{
  buffer_mutex_.lock();
  LoopCandidate loop_candidate;
  loop_candidate.index1 = msg->index1;
  loop_candidate.index2 = msg->index2;
  loop_candidate.pose(0, 3) = msg->pose.position.x;
  loop_candidate.pose(1, 3) = msg->pose.position.y;
  loop_candidate.pose(2, 3) = msg->pose.position.z;
  Eigen::Quaterniond q;
  q.x() = msg->pose.orientation.x;
  q.y() = msg->pose.orientation.y;
  q.z() = msg->pose.orientation.z;
  q.w() = msg->pose.orientation.w;
  loop_candidate.pose.block<3, 3>(0, 0) = q.matrix();
  buffer_.push_back(loop_candidate);
  buffer_mutex_.unlock();
}

void LoopCandidateSubscriber::parse_data(std::deque<LoopCandidate> & output)
{
  buffer_mutex_.lock();
  if (buffer_.size() > 0) {
    output.insert(output.end(), buffer_.begin(), buffer_.end());
    buffer_.clear();
  }
  buffer_mutex_.unlock();
}
}  // namespace slt_common
