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

#include "slt_common/publisher/lidar_frames_publisher.hpp"

namespace slt_common
{
LidarFramesPublisher::LidarFramesPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, int buff_size)
: node_(node)
{
  publisher_ =
    node_->create_publisher<slt_interface::msg::LidarFrames>(topic_name, buff_size);
}

void LidarFramesPublisher::publish(const std::vector<LidarFrame> & frames)
{
  slt_interface::msg::LidarFrames msg;
  for (auto & frame : frames) {
    slt_interface::msg::LidarFrame frame_msg;
    rclcpp::Time ros_time(static_cast<uint64_t>(frame.time * 1e9));
    frame_msg.header.stamp = ros_time;
    frame_msg.index = frame.index;
    frame_msg.pose.position.x = frame.pose(0, 3);
    frame_msg.pose.position.y = frame.pose(1, 3);
    frame_msg.pose.position.z = frame.pose(2, 3);
    Eigen::Quaterniond q(frame.pose.block<3, 3>(0, 0));
    frame_msg.pose.orientation.x = q.x();
    frame_msg.pose.orientation.y = q.y();
    frame_msg.pose.orientation.z = q.z();
    frame_msg.pose.orientation.w = q.w();
    msg.frames.push_back(frame_msg);
  }
  publisher_->publish(msg);
}

}  // namespace slt_common
