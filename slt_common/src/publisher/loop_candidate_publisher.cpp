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

#include "slt_common/publisher/loop_candidate_publisher.hpp"

namespace slt_common
{
LoopCandidatePublisher::LoopCandidatePublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, int buffer_size)
: node_(node)
{
  publisher_ =
    node_->create_publisher<slt_interface::msg::LoopCandidate>(topic_name, buffer_size);
}

void LoopCandidatePublisher::publish(LoopCandidate & loop_candidate)
{
  slt_interface::msg::LoopCandidate msg;

  msg.index1 = loop_candidate.index1;
  msg.index2 = loop_candidate.index2;
  msg.pose.position.x = loop_candidate.pose(0, 3);
  msg.pose.position.y = loop_candidate.pose(1, 3);
  msg.pose.position.z = loop_candidate.pose(2, 3);
  Eigen::Quaterniond q(loop_candidate.pose.block<3, 3>(0, 0));
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  publisher_->publish(msg);
}

}  // namespace slt_common
