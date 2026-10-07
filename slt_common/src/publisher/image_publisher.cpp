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

#include "slt_common/publisher/image_publisher.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "slt_common/msg_utils.hpp"

namespace slt_common
{

ImagePublisher::ImagePublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buffer_size)
: node_(node), frame_id_(frame_id)
{
  publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, buffer_size);
}

void ImagePublisher::publish(const ImageData & image_data)
{
  publish(image_data.image, image_data.time);
}

void ImagePublisher::publish(const cv::Mat & image, double time)
{
  if (image.empty()) {
    return;
  }
  std_msgs::msg::Header header;
  header.stamp = to_ros_time(time);
  header.frame_id = frame_id_;
  const std::string encoding = image.channels() == 3 ? "bgr8" : "mono8";
  publisher_->publish(*cv_bridge::CvImage(header, encoding, image).toImageMsg());
}

bool ImagePublisher::has_subscribers()
{
  return publisher_->get_subscription_count() > 0;
}

}  // namespace slt_common
