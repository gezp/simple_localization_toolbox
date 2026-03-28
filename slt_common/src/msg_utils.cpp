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

#include "slt_common/msg_utils.hpp"

namespace slt_common
{

rclcpp::Time to_ros_time(double time)
{
  return rclcpp::Time(static_cast<uint64_t>(time * 1e9));
}

geometry_msgs::msg::Transform to_transform_msg(Eigen::Matrix4d pose)
{
  geometry_msgs::msg::Transform msg;
  Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  msg.translation.x = pose(0, 3);
  msg.translation.y = pose(1, 3);
  msg.translation.z = pose(2, 3);
  msg.rotation.x = q.x();
  msg.rotation.y = q.y();
  msg.rotation.z = q.z();
  msg.rotation.w = q.w();
  return msg;
}

}  // namespace slt_common
