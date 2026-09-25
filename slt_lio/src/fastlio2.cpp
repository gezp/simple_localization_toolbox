// Copyright 2026 Zhenpeng Ge (https://github.com/zhenpengge).
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

#include "slt_lio/fastlio2.hpp"

#include <pcl/common/transforms.h>

#include <cmath>
#include <iostream>

#include "slt_common/lidar_utils.hpp"
#include "slt_lio/point_plane_residual.hpp"

namespace slt_lio
{

Fastlio2::Fastlio2(const YAML::Node & config)
{
  config_ = config;
  eskf_ = std::make_unique<Eskf>(config["eskf"]);
  input_filter_ = std::make_shared<slt_common::VoxelFilter>(config["input_filter"]);
  display_filter_ = std::make_shared<slt_common::VoxelFilter>(config["display_filter"]);
  initializer_ = std::make_unique<LioInitializer>(config["lio_initializer"]);
  num_nearby_ = config["correspondence_search"]["num_nearby"].as<int>();
  nearby_distance_ = config["correspondence_search"]["nearby_distance"].as<double>();
  plane_eigen_ratio_ = config["correspondence_search"]["plane_eigen_ratio"].as<double>();
  max_plane_distance_ = config["correspondence_search"]["max_plane_distance"].as<double>();
  max_num_iterations_ = config["eskf"]["max_num_iterations"].as<int>();
  map_crop_range_ = config["local_map"]["map_crop_range"].as<double>();
  double tree_resolution = config["local_map"]["tree_resolution"].as<double>();
  // (delete_param, balance_param, box_length) as FAST-LIO
  ikd_tree_ = std::make_unique<IkdTree>(0.3, 0.6, 0.2);
  ikd_tree_->set_downsample_param(tree_resolution);
  bool enable = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enable);
  elapsed_time_statistics_.set_title("Fastlio2");
}

Fastlio2::~Fastlio2()
{
}

void Fastlio2::set_extrinsic(
  const Eigen::Matrix4d & T_imu_lidar, const Eigen::Matrix4d & T_base_imu)
{
  T_imu_lidar_ = T_imu_lidar;
  T_base_imu_ = T_base_imu;
  R_imu_lidar_ = T_imu_lidar_.block<3, 3>(0, 0);
  t_imu_lidar_ = T_imu_lidar_.block<3, 1>(0, 3);
  has_extrinsic_ = true;
  initializer_->set_extrinsic(T_imu_lidar_);
}

void Fastlio2::add_imu_data(const slt_common::ImuData & imu_data)
{
  if (!is_inited_) {
    // the startup window seeds the state, never propagates: a gappy imu startup cannot
    // reach the filter
    initializer_->add_imu_data(imu_data);
    return;
  }
  imu_history_buffer_.emplace(imu_data.time, imu_data);
  while (imu_history_buffer_.size() > kMaxImuBuffer) {
    imu_history_buffer_.erase(imu_history_buffer_.begin());
  }
}

void Fastlio2::add_lidar_data(const slt_common::LidarData & lidar_data)
{
  if (!is_inited_) {
    initializer_->add_lidar_data(lidar_data);
  }
  lidar_buffer_.push_back(lidar_data);
  while (lidar_buffer_.size() > kMaxLidarBuffer) {
    lidar_buffer_.pop_front();
  }
}

bool Fastlio2::update()
{
  if (!has_extrinsic_) {
    return false;
  }
  if (!is_inited_ && !try_init()) {
    return false;
  }
  bool has_new_observation = false;
  // consume every observation the history already covers, oldest first
  while (!lidar_buffer_.empty()) {
    const auto & lidar_data = lidar_buffer_.front();
    const double t_obs = lidar_data.time + kScanDuration;
    // an observation older than the state does not describe it: drop it
    if (t_obs < eskf_->get_time()) {
      lidar_buffer_.pop_front();
      continue;
    }
    // the sample straddling the observation time is not here yet, wait for it
    if (imu_history_buffer_.empty() || imu_history_buffer_.rbegin()->first < t_obs) {
      break;
    }
    consume_observation(lidar_data, t_obs);
    lidar_buffer_.pop_front();
    has_new_observation = true;
  }
  return has_new_observation;
}

void Fastlio2::consume_observation(const slt_common::LidarData & lidar_data, double t_obs)
{
  elapsed_time_statistics_.tic("consume_observation");
  // the state must describe the scan: the buffered imu carries the filter to the
  // observation time, the sample there lands it exactly on it
  propagate_eskf_to(t_obs);
  if (eskf_->get_time() < t_obs) {
    eskf_->predict(sample_imu(t_obs));
  }
  // the scan is registered as a snapshot of its time (no per-point time to deskew with)
  current_scan_ = slt_common::to_pointcloud_xyz(lidar_data.point_cloud);
  pcl::transformPointCloud(*current_scan_, *current_scan_, T_imu_lidar_.cast<float>());
  has_new_scan_ = true;
  observe_point_cloud();
  elapsed_time_statistics_.toc("consume_observation");
  elapsed_time_statistics_.print_all_info("consume_observation", 20);
}

void Fastlio2::propagate_eskf_to(double time)
{
  for (auto it = imu_history_buffer_.upper_bound(eskf_->get_time());
       it != imu_history_buffer_.end() && it->first <= time; ++it) {
    eskf_->predict(it->second);
  }
}

slt_common::ImuData Fastlio2::sample_imu(double time)
{
  auto right = imu_history_buffer_.lower_bound(time);
  if (right == imu_history_buffer_.begin() || right == imu_history_buffer_.end()) {
    // `time` falls outside the history (imu dropout, or a backlog trimmed past it): the
    // nearest sample is all that is left, keep `time` so the caller's state lands on it
    slt_common::ImuData imu =
      right == imu_history_buffer_.end() ? std::prev(right)->second : right->second;
    imu.time = time;
    return imu;
  }
  return slt_common::interpolate_imu(std::prev(right)->second, right->second, time);
}

bool Fastlio2::try_init()
{
  // the seed imu sample is the one the state is defined at (state.time == imu_data.time)
  slt_common::ImuNavState state;
  slt_common::ImuData imu_data;
  if (!initializer_->try_init(state, imu_data)) {
    return false;
  }
  eskf_->init_state(state, imu_data);
  initializer_->reset();
  while (!lidar_buffer_.empty() && lidar_buffer_.front().time <= eskf_->get_time()) {
    lidar_buffer_.pop_front();
  }
  is_inited_ = true;
  std::cout << "Fastlio2: initialization done" << std::endl;
  return true;
}

bool Fastlio2::is_initialized() const
{
  return is_inited_;
}

slt_common::ImuNavState Fastlio2::get_imu_nav_state()
{
  return eskf_->get_imu_nav_state();
}

slt_common::ImuData Fastlio2::get_imu_data()
{
  return eskf_->get_imu_data();
}

Eigen::Matrix4d Fastlio2::get_current_pose()
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  const auto & state = eskf_->get_imu_nav_state();
  pose.block<3, 3>(0, 0) = state.orientation;
  pose.block<3, 1>(0, 3) = state.position;
  return pose;
}

Fastlio2::PointCloudPtr Fastlio2::get_current_scan()
{
  if (!has_new_scan_ || current_scan_ == nullptr) {
    return nullptr;
  }
  has_new_scan_ = false;
  auto scan = display_filter_->apply(current_scan_);
  pcl::transformPointCloud(*scan, *scan, get_current_pose());
  return scan;
}

bool Fastlio2::observe_point_cloud()
{
  auto input = input_filter_->apply(current_scan_);
  const auto & state = eskf_->get_imu_nav_state();
  Eigen::Matrix3d R = state.orientation;
  Eigen::Vector3d t = state.position;
  int num_observation = static_cast<int>(input->points.size());
  double sqr_nearby_distance = nearby_distance_ * nearby_distance_;
  for (int iter = 0; iter < max_num_iterations_; iter++) {
    std::vector<Eskf::PlaneObservation> observations;
    observations.reserve(num_observation);
    std::vector<float> search_sqr_distances;
    for (int i = 0; i < num_observation; i++) {
      Eigen::Vector3d p_body = input->points[i].getVector3fMap().cast<double>();
      Eigen::Vector3d p_world = R * p_body + t;
      pcl::PointXYZ p_map;
      p_map.getVector3fMap() = p_world.cast<float>();
      IkdTree::PointVector nearest_points;
      ikd_tree_->Nearest_Search(p_map, num_nearby_, nearest_points, search_sqr_distances);
      if (static_cast<int>(nearest_points.size()) < num_nearby_) {
        continue;
      }
      if (search_sqr_distances.back() > sqr_nearby_distance) {
        continue;
      }
      std::vector<Eigen::Vector3d> neighbors;
      neighbors.reserve(nearest_points.size());
      for (auto & np : nearest_points) {
        neighbors.push_back(np.getVector3fMap().cast<double>());
      }
      Eigen::Vector3d plane_point, plane_normal;
      if (!PlaneFit::fit(neighbors, plane_eigen_ratio_, plane_point, plane_normal)) {
        continue;
      }
      if (std::abs((p_world - plane_point).dot(plane_normal)) > max_plane_distance_) {
        continue;
      }
      Eskf::PlaneObservation observation;
      observation.point = p_body;
      observation.plane_point = plane_point;
      observation.plane_normal = plane_normal;
      observations.push_back(observation);
    }
    if (observations.empty()) {
      break;
    }
    if (!eskf_->observe_point_cloud(observations)) {
      break;
    }
    const auto & updated_state = eskf_->get_imu_nav_state();
    R = updated_state.orientation;
    t = updated_state.position;
  }
  update_local_map();
  has_new_local_map_ = true;
  return true;
}

Fastlio2::PointCloudPtr Fastlio2::get_local_map()
{
  if (!has_new_local_map_) {
    return nullptr;
  }
  has_new_local_map_ = false;
  auto local_map = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  IkdTree::PointVector points;
  ikd_tree_->flatten(ikd_tree_->Root_Node, points, NOT_RECORD);
  local_map->points.reserve(points.size());
  for (auto & p : points) {
    local_map->points.push_back(p);
  }
  local_map->width = points.size();
  local_map->height = 1;
  local_map->is_dense = true;
  return local_map;
}

bool Fastlio2::update_local_map()
{
  auto current_pose = get_current_pose();
  // sliding local cube (FAST-LIO laser_map_fov_segment): shift the cube by one
  // sliver & drop the points left outside when the pose comes close to an edge
  if (map_crop_range_ > 0.0) {
    if (!local_map_initialized_) {
      for (int i = 0; i < 3; i++) {
        local_map_points_.vertex_min[i] = current_pose(i, 3) - map_crop_range_;
        local_map_points_.vertex_max[i] = current_pose(i, 3) + map_crop_range_;
      }
      local_map_initialized_ = true;
    } else {
      // shift when half-way to the edge
      double move_threshold = 0.5 * map_crop_range_;
      std::vector<BoxPointType> crop_boxes;
      for (int i = 0; i < 3; i++) {
        double dist_min = std::abs(current_pose(i, 3) - local_map_points_.vertex_min[i]);
        double dist_max = std::abs(current_pose(i, 3) - local_map_points_.vertex_max[i]);
        double shift = map_crop_range_ - move_threshold;
        if (dist_min <= move_threshold) {
          BoxPointType box = local_map_points_;
          box.vertex_min[i] = local_map_points_.vertex_max[i] - shift;
          crop_boxes.push_back(box);
          local_map_points_.vertex_min[i] -= shift;
          local_map_points_.vertex_max[i] -= shift;
        } else if (dist_max <= move_threshold) {
          BoxPointType box = local_map_points_;
          box.vertex_max[i] = local_map_points_.vertex_min[i] + shift;
          crop_boxes.push_back(box);
          local_map_points_.vertex_min[i] += shift;
          local_map_points_.vertex_max[i] += shift;
        }
      }
      if (!crop_boxes.empty()) {
        ikd_tree_->Delete_Point_Boxes(crop_boxes);
        // drain the deleted-point storage: flatten() keeps appending to it, so
        // it grows without bound otherwise
        IkdTree::PointVector removed_points;
        ikd_tree_->acquire_removed_points(removed_points);
      }
    }
  }
  auto input = input_filter_->apply(current_scan_);
  Eigen::Affine3f T_world_imu = Eigen::Translation3f(current_pose.block<3, 1>(0, 3).cast<float>()) *
                                current_pose.block<3, 3>(0, 0).cast<float>();
  pcl::transformPointCloud(*input, *input, T_world_imu);
  IkdTree::PointVector points_to_add;
  points_to_add.reserve(input->points.size());
  for (auto & p : input->points) {
    points_to_add.push_back(p);
  }
  if (!points_to_add.empty()) {
    // Add_Points dereferences Root_Node->division_axis without a null check
    if (ikd_tree_->Root_Node == nullptr) {
      ikd_tree_->Build(points_to_add);
    } else {
      ikd_tree_->Add_Points(points_to_add, true);
    }
  }
  return true;
}

}  // namespace slt_lio
