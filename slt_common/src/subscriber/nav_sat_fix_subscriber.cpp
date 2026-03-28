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

#include "slt_common/subscriber/nav_sat_fix_subscriber.hpp"

namespace slt_common
{
NavSatFixSubscriber::NavSatFixSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size)
: node_(node)
{
  subscriber_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    topic_name, buffer_size,
    std::bind(&NavSatFixSubscriber::msg_callback, this, std::placeholders::_1));
}

void NavSatFixSubscriber::set_map_origin(double latitude, double longitude, double altitude)
{
  geo_converter_.Reset(latitude, longitude, altitude);
  origin_position_inited_ = true;
  RCLCPP_INFO(
    node_->get_logger(), "Set map origin for ENU position: (%lf, %lf, %lf)", latitude, longitude,
    altitude);
}

void NavSatFixSubscriber::msg_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  GnssData gnss_data;
  gnss_data.time = rclcpp::Time(msg->header.stamp).seconds();
  gnss_data.latitude = msg->latitude;
  gnss_data.longitude = msg->longitude;
  gnss_data.altitude = msg->altitude;
  gnss_data.status = GnssStatus::FIXED_SOLUTION;
  // convert gnss to local enu
  if (!origin_position_inited_) {
    RCLCPP_INFO(node_->get_logger(), "Use the first gnss data as map origin");
    set_map_origin(gnss_data.latitude, gnss_data.longitude, gnss_data.altitude);
  }
  double e, n, u;
  geo_converter_.Forward(gnss_data.latitude, gnss_data.longitude, gnss_data.altitude, e, n, u);
  gnss_data.antenna_position = Eigen::Vector3d(e, n, u);
  gnss_data.antenna_position_valid = true;
  // push to queue
  buffer_mutex_.lock();
  buffer_.push_back(gnss_data);
  buffer_mutex_.unlock();
}

void NavSatFixSubscriber::parse_data(std::deque<GnssData> & output)
{
  buffer_mutex_.lock();
  if (buffer_.size() > 0) {
    output.insert(output.end(), buffer_.begin(), buffer_.end());
    buffer_.clear();
  }
  buffer_mutex_.unlock();
}

}  // namespace slt_common
