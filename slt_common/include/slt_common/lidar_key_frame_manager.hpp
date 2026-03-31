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

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "slt_common/point_cloud_filter/voxel_filter.hpp"
#include "slt_common/sensor_data/lidar_frame.hpp"

namespace slt_common
{
class LidarKeyFrameManager
{
public:
  explicit LidarKeyFrameManager(const std::string & data_path);
  // key frames
  size_t add_key_frame(
    double time, Eigen::Matrix4d pose, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  bool update_key_frame(size_t index, Eigen::Matrix4d pose);
  LidarFrame get_key_frame(size_t index);
  const std::vector<LidarFrame> & get_key_frames();
  size_t get_key_frame_count();
  void reset(const std::vector<LidarFrame> & key_frames);
  // point cloud
  bool save_point_cloud(size_t index, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr load_point_cloud(size_t index);
  // pose
  bool save_key_frame_pose();
  bool load_key_frame_pose();
  // map
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_local_map(
    size_t start, size_t end, std::shared_ptr<VoxelFilter> filter = nullptr);
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_global_map(std::shared_ptr<VoxelFilter> filter = nullptr);
  bool save_global_map(std::shared_ptr<VoxelFilter> filter = nullptr);

private:
  std::string data_path_ = "";
  std::string key_frames_path_ = "";
  std::string map_path_ = "";

  std::vector<LidarFrame> key_frames_;
};
}  // namespace slt_common
