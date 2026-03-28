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

#include "slt_common/subscriber/gnss_subscriber.hpp"

namespace slt_common
{

GnssSubscriber::GnssSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<slt_interface::msg::GnssData>(
    topic_name, buffer_size, std::bind(&GnssSubscriber::msg_callback, this, std::placeholders::_1));
}

void GnssSubscriber::msg_callback(slt_interface::msg::GnssData::SharedPtr msg)
{
  GnssData gnss_data;
  gnss_data.time = rclcpp::Time(msg->header.stamp).seconds();
  gnss_data.status = static_cast<GnssStatus>(msg->status);
  gnss_data.longitude = msg->longitude;
  gnss_data.latitude = msg->latitude;
  gnss_data.altitude = msg->altitude;

  auto & pos = msg->antenna_position;
  gnss_data.antenna_position = Eigen::Vector3d(pos.x, pos.y, pos.z);
  gnss_data.antenna_position_valid = msg->antenna_position_valid;
  gnss_data.dual_antenna_heading = msg->dual_antenna_heading;
  gnss_data.dual_antenna_heading_valid = msg->dual_antenna_heading_valid;

  buffer_mutex_.lock();
  buffer_.push_back(gnss_data);
  buffer_mutex_.unlock();
}

void GnssSubscriber::parse_data(std::deque<GnssData> & output)
{
  buffer_mutex_.lock();
  if (buffer_.size() > 0) {
    output.insert(output.end(), buffer_.begin(), buffer_.end());
    buffer_.clear();
  }
  buffer_mutex_.unlock();
}

}  // namespace slt_common
