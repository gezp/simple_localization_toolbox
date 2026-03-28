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

#include "slt_common/lidar_key_frame_manager.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>

namespace slt_common
{
LidarKeyFrameManager::LidarKeyFrameManager(const std::string & data_path)
{
  data_path_ = data_path;
  key_frames_path_ = data_path_ + "/key_frames";
  map_path_ = data_path_ + "/map";
  std::filesystem::create_directory(key_frames_path_);
}

size_t LidarKeyFrameManager::add_key_frame(
  double time, Eigen::Matrix4d pose, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  LidarFrame key_frame;
  key_frame.time = time;
  key_frame.pose = pose;
  key_frame.index = key_frames_.size();
  key_frames_.push_back(key_frame);
  // save point cloud
  if (point_cloud) {
    save_point_cloud(key_frame.index, point_cloud);
  }
  return key_frame.index;
}

bool LidarKeyFrameManager::update_key_frame(size_t index, Eigen::Matrix4d pose)
{
  if (index >= key_frames_.size()) {
    return false;
  }
  key_frames_[index].pose = pose;
  return true;
}

void LidarKeyFrameManager::reset(const std::vector<LidarFrame> & key_frames)
{
  key_frames_ = key_frames;
}

const std::vector<LidarFrame> & LidarKeyFrameManager::get_key_frames()
{
  return key_frames_;
}

LidarFrame LidarKeyFrameManager::get_key_frame(size_t index)
{
  assert(index < key_frames_.size());
  return key_frames_[index];
}

size_t LidarKeyFrameManager::get_key_frame_count()
{
  return key_frames_.size();
}

bool LidarKeyFrameManager::save_point_cloud(
  size_t index, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  assert(index < key_frames_.size());
  std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(index) + ".pcd";
  pcl::io::savePCDFileBinary(file_path, *point_cloud);
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarKeyFrameManager::load_point_cloud(size_t index)
{
  assert(index < key_frames_.size());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(index) + ".pcd";
  pcl::io::loadPCDFile(file_path, *cloud);
  return cloud;
}

bool LidarKeyFrameManager::save_key_frame_pose()
{
  // open file
  std::ofstream ofs;
  std::string filename = data_path_ + "/key_frame_pose.txt";
  ofs.open(filename, std::ios::out | std::ios::trunc);
  if (!ofs) {
    std::cout << "failed to open path: " << filename << std::endl;
    return false;
  }
  // timestamp tx ty tz qx qy qz qw
  ofs << "# timestamp tx ty tz qx qy qz qw" << std::endl;
  for (size_t i = 0; i < key_frames_.size(); ++i) {
    auto & key_frame = key_frames_[i];
    Eigen::Vector3d t = key_frame.pose.block<3, 1>(0, 3);
    Eigen::Quaterniond q(key_frame.pose.block<3, 3>(0, 0));
    ofs << key_frame.time << " ";
    ofs << t.x() << " " << t.y() << " " << t.z() << " ";
    ofs << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  return true;
}

bool LidarKeyFrameManager::load_key_frame_pose()
{
  // open file
  std::string line;
  std::string filename = data_path_ + "/key_frame_pose.txt";
  std::ifstream ifs(filename);
  ifs.open(filename, std::ios::in);
  if (!ifs) {
    std::cout << "failed to open path: " << filename << std::endl;
    return false;
  }
  // get data
  double time;
  Eigen::Vector3d t;
  Eigen::Quaterniond q;
  while (getline(ifs, line)) {
    if (line.size() == 0 || line[0] == '#') {
      continue;
    }
    sscanf(
      line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &t.x(), &t.y(), &t.z(), &q.x(),
      &q.y(), &q.z(), &q.w());
    // create new frame
    LidarFrame key_frame;
    key_frame.time = time;
    key_frame.index = key_frames_.size();
    key_frame.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
    key_frame.pose.block<3, 1>(0, 3) = t;
    key_frames_.push_back(key_frame);
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarKeyFrameManager::get_local_map(
  size_t start, size_t end, std::shared_ptr<VoxelFilter> filter)
{
  if (start > end || end >= key_frames_.size()) {
    return nullptr;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = start; i <= end; ++i) {
    auto & key_frame = key_frames_[i];
    auto cloud = load_point_cloud(key_frame.index);
    pcl::transformPointCloud(*cloud, *cloud, key_frame.pose);
    *map_cloud += *cloud;
  }
  if (filter) {
    map_cloud = filter->apply(map_cloud);
  }
  return map_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarKeyFrameManager::get_global_map(
  std::shared_ptr<VoxelFilter> filter)
{
  return get_local_map(0, key_frames_.size() - 1, filter);
}

bool LidarKeyFrameManager::save_global_map(std::shared_ptr<VoxelFilter> filter)
{
  if (key_frames_.size() == 0) {
    return false;
  }
  std::filesystem::remove_all(map_path_);
  if (!std::filesystem::create_directory(map_path_)) {
    return false;
  }
  std::cout << "start to save map, key frames size:" << key_frames_.size() << std::endl;
  // global map
  auto global_map = get_global_map();
  std::string map_file_path = map_path_ + "/map.pcd";
  pcl::io::savePCDFileBinary(map_file_path, *global_map);
  // filter global map
  if (filter) {
    global_map = filter->apply(global_map);
    std::string filtered_map_file_path = map_path_ + "/filtered_map.pcd";
    pcl::io::savePCDFileBinary(filtered_map_file_path, *global_map);
  }
  //
  std::cout << "save map successfully, map dir:" << map_path_ << std::endl;
  return true;
}

}  // namespace slt_common
