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

#include "slt_common/loam/loam_feature_extraction.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// some references:
// https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/src/scanRegistration.cpp

namespace slt_common
{

LoamFeatureExtraction::LoamFeatureExtraction(const YAML::Node & config)
{
  num_rings_ = config["num_rings"].as<int>();
  num_regions_ = config["num_regions"].as<int>();
  //
  curvature_padding_ = config["curvature_padding"].as<int>();
  corner_curvature_threshold_ = config["corner_curvature_threshold"].as<double>();
  surface_curvature_threshold_ = config["surface_curvature_threshold"].as<double>();
  //
  neighbor_padding_ = config["neighbor_padding"].as<int>();
  neighbor_distance_threshold_ = config["neighbor_distance_threshold"].as<double>();
  //
  max_num_corner_sharp_ = config["max_num_corner_sharp"].as<int>();
  max_num_corner_less_sharp_ = config["max_num_corner_less_sharp"].as<int>();
  max_num_surface_flat_ = config["max_num_surface_flat"].as<int>();
  voxel_filter_leaf_size_ = config["voxel_filter_leaf_size"].as<double>();
  //
  corner_sharp_rgb_ = config["feature_color_rgb"]["corner_sharp"].as<std::vector<int>>();
  assert(corner_sharp_rgb_.size() == 3);
  corner_less_sharp_rgb_ = config["feature_color_rgb"]["corner_less_sharp"].as<std::vector<int>>();
  assert(corner_less_sharp_rgb_.size() == 3);
  surface_flat_rgb_ = config["feature_color_rgb"]["surface_flat"].as<std::vector<int>>();
  assert(surface_flat_rgb_.size() == 3);
  surface_less_flat_rgb_ = config["feature_color_rgb"]["surface_less_flat"].as<std::vector<int>>();
  assert(surface_less_flat_rgb_.size() == 3);
  debug_ = config["debug"].as<bool>();
}

bool LoamFeatureExtraction::extract(
  const pcl::PointCloud<PointXYZIRT>::Ptr & point_cloud, LoamFeature & feature)
{
  feature.corner_sharp.reset(new PointCloudType);
  feature.corner_less_sharp.reset(new PointCloudType);
  feature.surface_flat.reset(new PointCloudType);
  feature.surface_less_flat.reset(new PointCloudType);
  std::vector<PointCloudType> ring_point_clouds(num_rings_);
  // divide point cloud by ring id
  for (auto & p : point_cloud->points) {
    if (p.ring >= num_rings_ || std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      continue;
    }
    PointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    // use label to store ring id. (it's convenient to use built-in point cloud type)
    point.label = p.ring;
    ring_point_clouds[p.ring].emplace_back(point);
  }
  // extract feature for each ring
  for (int i = 0; i < num_rings_; i++) {
    auto & ring_point_cloud = ring_point_clouds[i];
    int point_size_in_ring = ring_point_clouds[i].points.size();
    if (point_size_in_ring < min_point_size_in_ring_) {
      continue;
    }
    // curvature, index sorted by curvature, picked_state for each point in current ring
    std::vector<double> point_curvature(point_size_in_ring, -1);
    std::vector<int> sorted_point_idx(point_size_in_ring, -1);
    std::vector<bool> point_picked_state(point_size_in_ring, false);
    // calculate curvature, need add padding.
    for (int j = curvature_padding_; j < point_size_in_ring - curvature_padding_; j++) {
      sorted_point_idx[j] = j;
      point_curvature[j] = get_point_curvature(ring_point_cloud, j);
    }
    // divide in some regions for more uniform feature
    int valid_point_size_in_ring = (point_size_in_ring - curvature_padding_ * 2);
    PointCloudPtr surface_less_flat_in_ring(new PointCloudType);
    for (int j = 0; j < num_regions_; j++) {
      int start_idx = curvature_padding_ + valid_point_size_in_ring * j / num_regions_;
      int end_idx = curvature_padding_ + valid_point_size_in_ring * (j + 1) / num_regions_ - 1;
      // sort sorted_point_idx by point_curvature
      auto cmp = [&](const int & a, const int & b) {
          return point_curvature[a] < point_curvature[b];
        };
      std::sort(sorted_point_idx.begin() + start_idx, sorted_point_idx.begin() + end_idx + 1, cmp);
      // sharp corner points and less sharp corner points
      int largest_picked_num = 0;
      for (int k = end_idx; k >= start_idx; k--) {
        int idx = sorted_point_idx[k];
        if (!point_picked_state[idx] && point_curvature[idx] > corner_curvature_threshold_) {
          largest_picked_num++;
          point_picked_state[idx] = true;
          if (largest_picked_num <= max_num_corner_sharp_) {
            feature.corner_sharp->push_back(ring_point_cloud.points[idx]);
          } else if (largest_picked_num <= max_num_corner_less_sharp_) {
            feature.corner_less_sharp->push_back(ring_point_cloud.points[idx]);
          } else {
            break;
          }
          mark_neighbor_points(ring_point_cloud, point_picked_state, idx);
        }
      }
      // flat surface points
      int smallest_picked_num = 0;
      for (int k = start_idx; k <= end_idx; k++) {
        int idx = sorted_point_idx[k];
        if (!point_picked_state[idx] && point_curvature[idx] < surface_curvature_threshold_) {
          smallest_picked_num++;
          point_picked_state[idx] = true;
          if (smallest_picked_num <= max_num_surface_flat_) {
            feature.surface_flat->push_back(ring_point_cloud.points[idx]);
          } else {
            break;
          }
          mark_neighbor_points(ring_point_cloud, point_picked_state, idx);
        }
      }
      // less_flat surface points
      for (int k = start_idx; k <= end_idx; k++) {
        int idx = sorted_point_idx[k];
        if (!point_picked_state[idx] && point_curvature[idx] < surface_curvature_threshold_) {
          surface_less_flat_in_ring->push_back(ring_point_cloud.points[idx]);
        }
      }
    }
    // downsample surface_less_flat_in_ring
    pcl::VoxelGrid<PointType> downsample_filter;
    downsample_filter.setInputCloud(surface_less_flat_in_ring);
    downsample_filter.setLeafSize(
      voxel_filter_leaf_size_, voxel_filter_leaf_size_, voxel_filter_leaf_size_);
    downsample_filter.filter(*surface_less_flat_in_ring);
    // append surface_less_flat
    *feature.surface_less_flat += *surface_less_flat_in_ring;
  }
  //
  if (debug_) {
    std::cout << "current loam feature: " << std::endl;
    print_feature_info(feature);
  }
  return true;
}

void LoamFeatureExtraction::print_feature_info(const LoamFeature & feature)
{
  std::cout << "corner_sharp number:" << feature.corner_sharp->points.size() << std::endl;
  std::cout << "corner_less_sharp number:" << feature.corner_less_sharp->points.size() << std::endl;
  std::cout << "surface_flat number:" << feature.surface_flat->points.size() << std::endl;
  std::cout << "surface_less_flat number:" << feature.surface_less_flat->points.size() << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoamFeatureExtraction::get_feature_point_cloud(
  const LoamFeature & feature)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto & p : feature.corner_sharp->points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = corner_sharp_rgb_[0];
    point.g = corner_sharp_rgb_[1];
    point.b = corner_sharp_rgb_[2];
    point_cloud->push_back(point);
  }
  for (auto & p : feature.corner_less_sharp->points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = corner_less_sharp_rgb_[0];
    point.g = corner_less_sharp_rgb_[1];
    point.b = corner_less_sharp_rgb_[2];
    point_cloud->push_back(point);
  }
  for (auto & p : feature.surface_flat->points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = surface_flat_rgb_[0];
    point.g = surface_flat_rgb_[1];
    point.b = surface_flat_rgb_[2];
    point_cloud->push_back(point);
  }
  for (auto & p : feature.surface_less_flat->points) {
    pcl::PointXYZRGB point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.r = surface_less_flat_rgb_[0];
    point.g = surface_less_flat_rgb_[1];
    point.b = surface_less_flat_rgb_[2];
    point_cloud->push_back(point);
  }
  return point_cloud;
}

LoamFeature LoamFeatureExtraction::transform_feature(
  const LoamFeature & input, const Eigen::Matrix4d & pose)
{
  LoamFeature output;
  output.corner_sharp.reset(new PointCloudType);
  output.corner_less_sharp.reset(new PointCloudType);
  output.surface_flat.reset(new PointCloudType);
  output.surface_less_flat.reset(new PointCloudType);
  if (input.corner_sharp->size() > 0) {
    pcl::transformPointCloud(*input.corner_sharp, *output.corner_sharp, pose);
  }
  if (input.corner_less_sharp->size() > 0) {
    pcl::transformPointCloud(*input.corner_less_sharp, *output.corner_less_sharp, pose);
  }
  if (input.surface_flat->size() > 0) {
    pcl::transformPointCloud(*input.surface_flat, *output.surface_flat, pose);
  }
  if (input.surface_less_flat->size() > 0) {
    pcl::transformPointCloud(*input.surface_less_flat, *output.surface_less_flat, pose);
  }
  return output;
}

void LoamFeatureExtraction::mark_neighbor_points(
  const PointCloudType & ring_point_cloud, std::vector<bool> & mark, int idx)
{
  // mark neighbor point
  for (int k = 1; k <= neighbor_padding_; k++) {
    if (idx + k >= static_cast<int>(ring_point_cloud.points.size())) {
      break;
    }
    double diff_x = ring_point_cloud.points[idx + k].x - ring_point_cloud.points[idx + k - 1].x;
    double diff_y = ring_point_cloud.points[idx + k].y - ring_point_cloud.points[idx + k - 1].y;
    double diff_z = ring_point_cloud.points[idx + k].z - ring_point_cloud.points[idx + k - 1].z;
    // break if it's discontinuous
    if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > neighbor_distance_threshold_) {
      break;
    }
    mark[idx + k] = true;
  }
  for (int k = -1; k >= -neighbor_padding_; k--) {
    if (idx + k < 0) {
      break;
    }
    double diff_x = ring_point_cloud.points[idx + k].x - ring_point_cloud.points[idx + k + 1].x;
    double diff_y = ring_point_cloud.points[idx + k].y - ring_point_cloud.points[idx + k + 1].y;
    double diff_z = ring_point_cloud.points[idx + k].z - ring_point_cloud.points[idx + k + 1].z;
    if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > neighbor_distance_threshold_) {
      break;
    }
    mark[idx + k] = true;
  }
}

double LoamFeatureExtraction::get_point_curvature(const PointCloudType & ring_point_cloud, int idx)
{
  double diff_x = 0;
  double diff_y = 0;
  double diff_z = 0;
  for (int i = -curvature_padding_; i <= curvature_padding_; i++) {
    diff_x += ring_point_cloud.points[idx + i].x - ring_point_cloud.points[idx].x;
    diff_y += ring_point_cloud.points[idx + i].y - ring_point_cloud.points[idx].y;
    diff_z += ring_point_cloud.points[idx + i].z - ring_point_cloud.points[idx].z;
  }
  return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
}

}  // namespace slt_common
