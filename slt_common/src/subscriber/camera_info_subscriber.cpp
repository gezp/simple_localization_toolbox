// Copyright 2026 Gezp (https://github.com/gezp).
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

#include "slt_common/subscriber/camera_info_subscriber.hpp"

namespace slt_common
{

CameraInfoSubscriber::CameraInfoSubscriber(rclcpp::Node::SharedPtr node, std::string topic_name)
: node_(node)
{
  subscriber_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    topic_name, rclcpp::QoS(1).transient_local(),
    std::bind(&CameraInfoSubscriber::msg_callback, this, std::placeholders::_1));
}

void CameraInfoSubscriber::msg_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_ptr)
{
  CameraIntrinsicParam param;
  param.frame_id = camera_info_msg_ptr->header.frame_id;
  param.height = camera_info_msg_ptr->height;
  param.width = camera_info_msg_ptr->width;
  param.distortion_model = camera_info_msg_ptr->distortion_model;
  // K is the row-major intrinsic matrix; only fx, fy, cx, cy are kept
  if (camera_info_msg_ptr->k.size() >= 9) {
    const auto & k = camera_info_msg_ptr->k;
    param.intrinsics = {k[0], k[4], k[2], k[5]};
  }
  param.distortion_coeffs = camera_info_msg_ptr->d;

  param_mutex_.lock();
  param_ = param;
  has_param_ = true;
  param_mutex_.unlock();
}

bool CameraInfoSubscriber::get_param(CameraIntrinsicParam & output)
{
  param_mutex_.lock();
  bool has_param = has_param_;
  if (has_param) {
    output = param_;
  }
  param_mutex_.unlock();
  return has_param;
}

}  // namespace slt_common
