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

#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "slt_common/sensor_data/image_data.hpp"

namespace slt_common
{

class ImagePublisher
{
public:
  ImagePublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buffer_size);
  ~ImagePublisher() = default;

  void publish(const ImageData & image_data);
  // the encoding is taken from the image; an empty image is dropped
  void publish(const cv::Mat & image, double time);
  bool has_subscribers();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::string frame_id_;
};
}  // namespace slt_common
