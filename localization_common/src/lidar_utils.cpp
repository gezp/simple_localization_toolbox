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

#include "localization_common/lidar_utils.hpp"

#include <memory>
#include <vector>

namespace localization_common
{

template<typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(const Eigen::Matrix<T, 3, 1> & angle_axis)
{
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(), quaternion_xyz.z());
}

Eigen::Matrix4d integrate_twist(const TwistData & twist_data, double dt, bool use_2d_plane)
{
  Eigen::Matrix4d T;
  // approximately integrate, only for small delta time
  if (use_2d_plane) {
    Eigen::AngleAxisd t_Vz(twist_data.angular_velocity.z() * dt, Eigen::Vector3d::UnitZ());
    T.block<3, 3>(0, 0) = t_Vz.toRotationMatrix();
    T.block<3, 1>(0, 3) = twist_data.linear_velocity * dt;
    T(2, 3) = 0;
  } else {
    Eigen::Vector3d angle = twist_data.angular_velocity * dt;
    T.block<3, 3>(0, 0) = AngleAxisVectorToRotationQuaternion(angle).toRotationMatrix();
    T.block<3, 1>(0, 3) = twist_data.linear_velocity * dt;
  }
  return T;
}

bool convert_velodyne64(
  const LidarData<pcl::PointXYZI> & data1, LidarData<PointXYZIRT> & data2, double dt,
  bool is_clockwise)
{
  int num_scans = 64;
  data2.time = data1.time;
  data2.point_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
  data2.point_cloud->points.resize(data1.point_cloud->points.size());
  // copy point cloud
  size_t cnt = 0;
  for (size_t i = 0; i < data1.point_cloud->points.size(); i++) {
    auto & p = data1.point_cloud->points[i];
    // calculate ring id for each point
    float angle = 180.0f / M_PI * atan(p.z / hypot(p.x, p.y));
    int laser_id = 0;
    if (angle >= -8.83) {
      laser_id = static_cast<int>((2 - angle) * 3.0 + 0.5);
    } else {
      laser_id = num_scans / 2 + static_cast<int>((-8.83 - angle) * 2.0 + 0.5);
    }
    // skip outlies, only use [0, 50]
    if (angle > 2 || angle < -24.33 || laser_id > 50 || laser_id < 0) {
      continue;
    }
    // skip too close points
    if (p.getVector3fMap().norm() < 2.0) {
      continue;
    }
    // copy point
    PointXYZIRT dst;
    dst.x = p.x;
    dst.y = p.y;
    dst.z = p.z;
    dst.intensity = p.intensity;
    dst.ring = laser_id;
    dst.time = 0;
    data2.point_cloud->points[cnt] = dst;
    cnt++;
  }
  data2.point_cloud->points.resize(cnt);
  // calculate time for each point
  std::vector<bool> is_first(num_scans, true);
  std::vector<double> yaw_first(num_scans, 0.0);  // yaw of first scan point
  for (size_t i = 0; i < data2.point_cloud->points.size(); i++) {
    auto & p = data2.point_cloud->points[i];
    int laser_id = p.ring;
    double yaw_angle = atan2(p.y, p.x);
    if (is_first[laser_id]) {
      yaw_first[laser_id] = yaw_angle;
      is_first[laser_id] = false;
      p.time = data2.time;
      continue;
    }
    double yaw_offset = yaw_angle - yaw_first[laser_id];
    if (is_clockwise) {
      yaw_offset = -yaw_offset;
    }
    if (yaw_offset < 0) {
      yaw_offset = yaw_offset + 2 * M_PI;
    }
    assert(yaw_offset >= 0 && yaw_offset <= 2 * M_PI);
    p.time = data2.time + yaw_offset / (2 * M_PI) * dt;
  }
  return true;
}

bool undistort_point_cloud(LidarData<PointXYZIRT> & lidar_data, const TwistData & twist_data)
{
  for (size_t i = 0; i < lidar_data.point_cloud->points.size(); i++) {
    auto & p = lidar_data.point_cloud->points[i];
    Eigen::Vector3d point(p.x, p.y, p.z);
    // lidar motion
    Eigen::Matrix4d T = integrate_twist(twist_data, p.time - lidar_data.time, false);
    // Rp + t
    Eigen::Vector3d undistort_point = T.block<3, 3>(0, 0) * point + T.block<3, 1>(0, 3);
    // update point xyz
    p.x = undistort_point.x();
    p.y = undistort_point.y();
    p.z = undistort_point.z();
  }
  return true;
}

}  // namespace localization_common
