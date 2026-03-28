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

#include "slt_common/extrinsics_manager.hpp"

#include "tf2_eigen/tf2_eigen.hpp"

namespace slt_common
{
ExtrinsicsManager::ExtrinsicsManager(rclcpp::Node::SharedPtr node)
{
  // tf
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
}

void ExtrinsicsManager::enable_tf_listener()
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ExtrinsicsManager::disable_tf_listener() {tf_listener_.reset();}

bool ExtrinsicsManager::lookup(
  const std::string & base_frame_id, const std::string & child_frame_id, Eigen::Matrix4d & pose)
{
  if (base_frame_id == child_frame_id) {
    pose = Eigen::Matrix4d::Identity();
    return true;
  }
  try {
    auto transform = tf_buffer_->lookupTransform(base_frame_id, child_frame_id, tf2::TimePointZero);
    pose = tf2::transformToEigen(transform).matrix();
    return true;
  } catch (const tf2::TransformException & ex) {
    return false;
  }
  return true;
}

}  // namespace slt_common
