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

#include "slt_common/publisher/gnss_publisher.hpp"

namespace slt_common
{

GnssPublisher::GnssPublisher(rclcpp::Node::SharedPtr node, std::string topic_name, int buffer_size)
: node_(node)
{
  publisher_ =
    node_->create_publisher<slt_interface::msg::GnssData>(topic_name, buffer_size);
}

void GnssPublisher::publish(const GnssData & gnss_data)
{
  slt_interface::msg::GnssData msg;
  rclcpp::Time ros_time(static_cast<uint64_t>(gnss_data.time * 1e9));
  msg.header.stamp = ros_time;
  msg.status = static_cast<int>(gnss_data.status);
  msg.longitude = gnss_data.longitude;
  msg.latitude = gnss_data.latitude;
  msg.altitude = gnss_data.altitude;
  msg.antenna_position.x = gnss_data.antenna_position.x();
  msg.antenna_position.y = gnss_data.antenna_position.y();
  msg.antenna_position.z = gnss_data.antenna_position.z();
  msg.antenna_position_valid = gnss_data.antenna_position_valid;
  msg.dual_antenna_heading = gnss_data.dual_antenna_heading;
  msg.dual_antenna_heading_valid = gnss_data.dual_antenna_heading_valid;
  publisher_->publish(msg);
}

bool GnssPublisher::has_subscribers() {return publisher_->get_subscription_count() > 0;}

}  // namespace slt_common
