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

#include "slt_lidar_mapping/lio_back_end.hpp"

#include "slt_common/sensor_data_utils.hpp"
#include "slt_common/lidar_utils.hpp"

namespace slt_lidar_mapping
{

bool LioBackEnd::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_param
  key_frame_distance_ = config_node["key_frame_distance"].as<float>();
  gravity_magnitude_ = config_node["earth"]["gravity_magnitude"].as<double>();
  use_gnss_ = config_node["use_gnss"].as<bool>();
  use_loop_closure_ = config_node["use_loop_closure"].as<bool>();
  use_imu_pre_integration_ = config_node["use_imu_pre_integration"].as<bool>();
  optimize_step_with_key_frame = (config_node["optimization_step_size"]["key_frame"].as<int>());
  optimize_step_with_loop = (config_node["optimization_step_size"]["loop_closure"].as<int>());
  // init graph optimizer
  init_graph_optimizer(config_node);
  // init key_frame_manager
  key_frame_manager_ = std::make_shared<slt_common::LidarKeyFrameManager>(data_path);
  // init filter
  using VoxelFilter = slt_common::VoxelFilter;
  display_filter_ = std::make_shared<VoxelFilter>(config_node["display_filter"]);
  global_map_filter_ = std::make_shared<VoxelFilter>(config_node["global_map_filter"]);
  // init buffer
  gnss_odom_buffer_ = std::make_shared<slt_common::OdomDataBuffer>(10000);
  // print info
  std::cout << "display filter:" << std::endl;
  display_filter_->print_info();
  std::cout << "global_map filter:" << std::endl;
  global_map_filter_->print_info();
  return true;
}

void LioBackEnd::set_extrinsic(
  const Eigen::Matrix4d & T_base_imu, const Eigen::Matrix4d & T_lidar_imu)
{
  T_base_imu_ = T_base_imu;
  T_lidar_imu_ = T_lidar_imu;
  T_imu_lidar_ = T_lidar_imu.inverse();
  T_base_lidar_ = T_base_imu * T_lidar_imu.inverse();
}

bool LioBackEnd::add_imu_data(const slt_common::ImuData & imu)
{
  if (!imu_buffer_.empty() && imu.time <= imu_buffer_.back().time) {
    std::cout << "receive a early imu data" << std::endl;
    return false;
  }
  imu_buffer_.push_back(imu);
  if (imu_buffer_.size() > 10000) {
    std::cout << "too many old imu in buffer, drop imu data" << std::endl;
    imu_buffer_.pop_front();
  }
  return true;
}

bool LioBackEnd::add_gnss_odom(const slt_common::OdomData & gnss_odom)
{
  gnss_odom_buffer_->add_data(gnss_odom);
  return true;
}

bool LioBackEnd::add_loop_candidate(const slt_common::LoopCandidate & loop_candidate)
{
  if (!use_loop_closure_) {
    return false;
  }
  Eigen::Matrix4d relative_pose = T_lidar_imu_.inverse() * loop_candidate.pose * T_lidar_imu_;
  // add constraint loop closure detection:
  graph_optimizer_->add_relative_pose_edge(
    loop_candidate.index1, loop_candidate.index2, relative_pose, loop_closure_noise_);
  new_loop_cnt_++;
  return true;
}

bool LioBackEnd::update(
  const slt_common::LidarData & lidar_data, const slt_common::OdomData & lidar_odom)
{
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
  current_lidar_odom_ = lidar_odom;
  if (check_new_key_frame(lidar_odom)) {
    has_new_key_frame_ = true;
    // add new key_frame
    Eigen::Matrix4d pose = T_map_odom_ * lidar_odom.pose * T_base_lidar_;
    auto cloud_xyz = to_pointcloud_xyz(lidar_data);
    key_frame_manager_->add_key_frame(lidar_odom.time, pose, cloud_xyz);
    new_key_frame_cnt_++;
    // add node
    add_node_and_edge();
    if (optimize(false)) {
      has_new_optimized_ = true;
    }
    latest_key_lidar_odom_ = lidar_odom;
    return true;
  }
  return false;
}

bool LioBackEnd::optimize(bool force)
{
  if (
    (!force) && (new_key_frame_cnt_ < optimize_step_with_key_frame) &&
    (new_loop_cnt_ < optimize_step_with_loop))
  {
    return false;
  }
  // optimize
  if (!graph_optimizer_->optimize()) {
    return false;
  }
  // reset key frame counters:
  new_key_frame_cnt_ = new_loop_cnt_ = 0;
  // update key frames
  auto optimized_states = graph_optimizer_->get_optimized_vertices();
  assert(optimized_states.size() == key_frame_manager_->get_key_frame_count());
  for (size_t i = 0; i < optimized_states.size(); ++i) {
    Eigen::Matrix4d imu_pose = Eigen::Matrix4d::Identity();
    imu_pose.block<3, 1>(0, 3) = optimized_states[i].position;
    imu_pose.block<3, 3>(0, 0) = optimized_states[i].orientation;
    Eigen::Matrix4d pose = imu_pose * T_imu_lidar_;
    key_frame_manager_->update_key_frame(i, pose);
  }
  // update T_map_odom_
  T_map_odom_ = key_frame_manager_->get_key_frames().back().pose *
    (current_lidar_odom_.pose * T_base_lidar_).inverse();
  return true;
}

bool LioBackEnd::has_new_key_frame()
{
  return has_new_key_frame_;
}

bool LioBackEnd::has_new_optimized()
{
  return has_new_optimized_;
}

slt_common::OdomData LioBackEnd::get_current_odom()
{
  slt_common::OdomData odom;
  odom.time = current_lidar_odom_.time;
  odom.pose = T_map_odom_ * current_lidar_odom_.pose;
  return odom;
}

const std::vector<slt_common::LidarFrame> & LioBackEnd::get_key_frames()
{
  return key_frame_manager_->get_key_frames();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LioBackEnd::get_global_map()
{
  return key_frame_manager_->get_global_map(display_filter_);
}

bool LioBackEnd::save_map()
{
  key_frame_manager_->save_key_frame_pose();
  key_frame_manager_->save_global_map(global_map_filter_);
  return true;
}

bool LioBackEnd::init_graph_optimizer(const YAML::Node & config_node)
{
  std::string graph_optimizer_method = config_node["graph_optimizer_method"].as<std::string>();
  if (graph_optimizer_method == "g2o") {
    auto g2o_optimizer =
      std::make_shared<G2oGraphOptimizerPRVAG>(config_node[graph_optimizer_method]);
    Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -gravity_magnitude_);
    g2o_optimizer->set_gravity(gravity);
    graph_optimizer_ = g2o_optimizer;
  } else {
    std::cerr << "Optimizer " << graph_optimizer_method << " NOT FOUND!";
    return false;
  }
  std::cout << "\tOptimizer:" << graph_optimizer_method << std::endl << std::endl;
  lidar_odom_noise_.resize(6);
  loop_closure_noise_.resize(6);
  gnss_noise_.resize(3);
  // x-y-z & yaw-roll-pitch
  for (int i = 0; i < 6; ++i) {
    lidar_odom_noise_(i) = config_node[graph_optimizer_method]["lidar_odom_noise"][i].as<double>();
    loop_closure_noise_(i) =
      config_node[graph_optimizer_method]["loop_closure_noise"][i].as<double>();
  }
  // x-y-z:
  for (int i = 0; i < 3; i++) {
    gnss_noise_(i) = config_node[graph_optimizer_method]["gnss_noise"][i].as<double>();
  }
  return true;
}

bool LioBackEnd::check_new_key_frame(const slt_common::OdomData & lidar_odom)
{
  if (key_frame_manager_->get_key_frame_count() == 0) {
    return true;
  }
  Eigen::Vector3d translation =
    (lidar_odom.pose.block<3, 1>(0, 3) - latest_key_lidar_odom_.pose.block<3, 1>(0, 3));
  // whether the current scan is far away enough from last key frame:
  if (translation.lpNorm<1>() > key_frame_distance_) {
    return true;
  }
  return false;
}

bool LioBackEnd::get_synced_imu_buffer(double time, std::vector<slt_common::ImuData> & buffer)
{
  if (imu_buffer_.empty() || imu_buffer_.back().time < time) {
    return false;
  }
  slt_common::ImuData imu;
  while (imu_buffer_.front().time <= time) {
    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    buffer.push_back(imu);
  }
  // get sync imu
  if (imu.time != time) {
    imu = slt_common::interpolate_imu(imu, imu_buffer_.front(), time);
    buffer.push_back(imu);
  }
  return true;
}

bool LioBackEnd::add_node_and_edge()
{
  // add node for new key frame pose:
  // fix the pose of the first key frame for lidar only mapping:
  slt_common::ImuNavState imu_nav_state;
  auto keyframe = key_frame_manager_->get_key_frames().back();
  // get synced gnss odometry
  // TODO(all): check gnss valid
  slt_common::OdomData current_gnss_odom;
  if (!gnss_odom_buffer_->get_interpolated_data(keyframe.time, current_gnss_odom)) {
    std::cout << "fail to get synced gnss odom" << std::endl;
  }
  if (!use_gnss_ && graph_optimizer_->get_vertex_num() == 0) {
    imu_nav_state.time = keyframe.time;
    Eigen::Matrix4d pose = current_lidar_odom_.pose * T_base_imu_;
    imu_nav_state.position = pose.block<3, 1>(0, 3);
    imu_nav_state.orientation = pose.block<3, 3>(0, 0);
    graph_optimizer_->add_vertex(imu_nav_state, true);
  } else {
    auto odom = slt_common::transform_odom(current_gnss_odom, T_base_imu_.inverse());
    imu_nav_state.time = keyframe.time;
    imu_nav_state.position = odom.pose.block<3, 1>(0, 3);
    imu_nav_state.orientation = odom.pose.block<3, 3>(0, 0);
    imu_nav_state.linear_velocity = odom.linear_velocity;
    graph_optimizer_->add_vertex(imu_nav_state, false);
  }
  // add constraints:
  // lidar frontend
  int n = graph_optimizer_->get_vertex_num();
  if (n > 1) {
    Eigen::Matrix4d last_pose = latest_key_lidar_odom_.pose * T_base_imu_;
    Eigen::Matrix4d cur_pose = current_lidar_odom_.pose * T_base_imu_;
    Eigen::Matrix4d relative_pose = last_pose.inverse() * cur_pose;
    graph_optimizer_->add_relative_pose_edge(n - 2, n - 1, relative_pose, lidar_odom_noise_);
  }
  // gnss odometry
  if (use_gnss_) {
    Eigen::Vector3d pos = (current_gnss_odom.pose * T_base_imu_).block<3, 1>(0, 3);
    // add constraint, GNSS position:
    graph_optimizer_->add_prior_position_edge(n - 1, pos, gnss_noise_);
  }
  // imu pre-integration
  if (use_imu_pre_integration_) {
    // get synced pre-integrated imu_buffer
    std::vector<slt_common::ImuData> pre_integration_buffer;
    // TODO(all): check imu valid
    if (n == 1) {
      // get lastest_key_imu
      get_synced_imu_buffer(keyframe.time, pre_integration_buffer);
      lastest_key_imu_ = pre_integration_buffer.back();
    } else {
      pre_integration_buffer.push_back(lastest_key_imu_);
      get_synced_imu_buffer(keyframe.time, pre_integration_buffer);
      lastest_key_imu_ = pre_integration_buffer.back();
      // add constraint, IMU pre-integraion:
      graph_optimizer_->add_imu_pre_integration_edge(n - 2, n - 1, pre_integration_buffer);
    }
  }
  return true;
}

}  // namespace slt_lidar_mapping
