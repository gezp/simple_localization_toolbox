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

#include "slt_common/subscriber/cloud_subscriber.hpp"

#include <pcl/console/print.h>
#include <pcl_conversions/pcl_conversions.h>

#include "slt_common/lidar_utils.hpp"

namespace slt_common
{

CloudSubscriber::CloudSubscriber(
  rclcpp::Node::SharedPtr node, std::string topic_name, size_t buffer_size)
: node_(node)
{
  auto msg_callback = [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // TODO(gezp): fix rosbag.
      for (size_t i = 0; i < msg->fields.size(); i++) {
        if (msg->fields[i].name == "i") {
          msg->fields[i].name = "intensity";
        }
      }
      LidarData data;
      data.time = rclcpp::Time(msg->header.stamp).seconds();
      data.point_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
      auto prev_level = pcl::console::getVerbosityLevel();
      pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
      pcl::fromROSMsg(*msg, *(data.point_cloud));
      pcl::console::setVerbosityLevel(prev_level);
      remove_nan_from_pointcloud(data);
      data.has_intensity = has_field(msg, "intensity");
      data.has_ring = has_field(msg, "ring");
      data.has_time = has_field(msg, "time");
      buffer_mutex_.lock();
      buffer_.push_back(data);
      buffer_mutex_.unlock();
    };
  subscriber_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name, buffer_size, msg_callback);
}

void CloudSubscriber::parse_data(std::deque<LidarData> & output)
{
  buffer_mutex_.lock();
  if (buffer_.size() > 0) {
    output.insert(output.end(), buffer_.begin(), buffer_.end());
    buffer_.clear();
  }
  buffer_mutex_.unlock();
}

bool CloudSubscriber::has_field(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg, const std::string & field_name)
{
  for (size_t i = 0; i < msg->fields.size(); i++) {
    if (msg->fields[i].name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace slt_common
