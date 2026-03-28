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

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "slt_common/sensor_data/lidar_data.hpp"

namespace slt_common
{

class CloudPublisher
{
public:
  CloudPublisher(
    rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buffer_size);
  ~CloudPublisher() = default;

  template<typename PointT>
  void publish(const pcl::PointCloud<PointT> & cloud, rclcpp::Time time)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = time;
    msg.header.frame_id = frame_id_;
    publisher_->publish(msg);
  }

  template<typename PointT>
  void publish(const pcl::PointCloud<PointT> & cloud, double time)
  {
    rclcpp::Time ros_time(static_cast<uint64_t>(time * 1e9));
    publish(cloud, ros_time);
  }

  template<typename PointT>
  void publish(const pcl::PointCloud<PointT> & cloud)
  {
    publish(cloud, node_->get_clock()->now());
  }

  template<typename PointT>
  void publish(const LidarData<PointT> & lidar_data)
  {
    publish(*lidar_data.point_cloud, lidar_data.time);
  }

  bool has_subscribers();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::string frame_id_;
};
}  // namespace slt_common
