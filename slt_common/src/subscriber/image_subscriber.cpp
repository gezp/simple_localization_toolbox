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

#include "slt_common/subscriber/image_subscriber.hpp"

#include "cv_bridge/cv_bridge.hpp"

namespace slt_common
{

ImageSubscriber::ImageSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buff_size, bool enable_compressed)
: node_(node), enable_compressed_(enable_compressed), buffer_size_(buff_size)
{
  if (enable_compressed_) {
    compressed_subscriber_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_name, buff_size,
      std::bind(&ImageSubscriber::compressed_msg_callback, this, std::placeholders::_1));
  } else {
    subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic_name, buff_size,
        std::bind(&ImageSubscriber::msg_callback, this, std::placeholders::_1));
  }
}

std::string ImageSubscriber::get_topic_name() const
{
  if (enable_compressed_) {
    return compressed_subscriber_->get_topic_name();
  }
  return subscriber_->get_topic_name();
}

void ImageSubscriber::msg_callback(const sensor_msgs::msg::Image::SharedPtr image_msg_ptr)
{
  ImageData data;
  data.time = rclcpp::Time(image_msg_ptr->header.stamp).seconds();
  data.image = cv_bridge::toCvCopy(*image_msg_ptr)->image;

  buffer_mutex_.lock();
  data_buffer_.push_back(data);
  if (data_buffer_.size() > buffer_size_) {
    data_buffer_.pop_front();
  }
  buffer_mutex_.unlock();
}

void ImageSubscriber::compressed_msg_callback(
  const sensor_msgs::msg::CompressedImage::SharedPtr image_msg_ptr)
{
  ImageData data;
  data.time = rclcpp::Time(image_msg_ptr->header.stamp).seconds();
  data.image = cv_bridge::toCvCopy(*image_msg_ptr)->image;

  buffer_mutex_.lock();
  data_buffer_.push_back(data);
  if (data_buffer_.size() > buffer_size_) {
    data_buffer_.pop_front();
  }
  buffer_mutex_.unlock();
}

void ImageSubscriber::parse_data(std::deque<ImageData> & output)
{
  buffer_mutex_.lock();
  if (data_buffer_.size() > 0) {
    output.insert(output.end(), data_buffer_.begin(), data_buffer_.end());
    data_buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void ImageSubscriber::parse_data(std::map<double, ImageData> & output)
{
  buffer_mutex_.lock();
  if (data_buffer_.size() > 0) {
    for (auto & data : data_buffer_) {
      output.insert({data.time, data});
    }
    data_buffer_.clear();
  }
  buffer_mutex_.unlock();
}

void ImageSubscriber::clear()
{
  buffer_mutex_.lock();
  data_buffer_.clear();
  buffer_mutex_.unlock();
}

}  // namespace slt_common
