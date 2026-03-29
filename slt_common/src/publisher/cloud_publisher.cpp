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

#include "slt_common/publisher/cloud_publisher.hpp"

namespace slt_common
{

struct PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace slt_common

POINT_CLOUD_REGISTER_POINT_STRUCT(
  slt_common::PointXYZIT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, time, time))

namespace slt_common
{

CloudPublisher::CloudPublisher(
  rclcpp::Node::SharedPtr node, std::string topic_name, std::string frame_id, size_t buffer_size)
: node_(node), frame_id_(frame_id)
{
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, buffer_size);
}

bool CloudPublisher::has_subscribers()
{
  return publisher_->get_subscription_count() > 0;
}

void CloudPublisher::publish(const LidarData & lidar_data)
{
  if (!lidar_data.has_intensity) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::copyPointCloud(*lidar_data.point_cloud, cloud);
    publish(cloud, lidar_data.time);
  } else if (!lidar_data.has_time) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::copyPointCloud(*lidar_data.point_cloud, cloud);
    publish(cloud, lidar_data.time);
  } else if (!lidar_data.has_ring) {
    pcl::PointCloud<PointXYZIT> cloud;
    pcl::copyPointCloud(*lidar_data.point_cloud, cloud);
    publish(cloud, lidar_data.time);
  } else {
    publish(*lidar_data.point_cloud, lidar_data.time);
  }
}

}  // namespace slt_common
