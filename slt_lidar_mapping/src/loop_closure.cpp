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

#include "slt_lidar_mapping/loop_closure.hpp"

#include <filesystem>

namespace slt_lidar_mapping
{
LoopClosure::LoopClosure()
{
  registration_factory_ = std::make_shared<slt_common::CloudRegistrationFactory>();
}

bool LoopClosure::init_config(const std::string & config_path, const std::string & data_path)
{
  YAML::Node config_node = YAML::LoadFile(config_path);
  // init_data_path
  data_path_ = data_path;
  scan_context_path_ = data_path_ + "/scan_context";
  // init_param
  extend_frame_num_ = config_node["extend_frame_num"].as<int>();
  loop_step_ = config_node["loop_step"].as<int>();
  diff_num_ = config_node["diff_num"].as<int>();
  detect_area_ = config_node["detect_area"].as<float>();
  fitness_score_limit_ = config_node["fitness_score_limit"].as<float>();
  use_same_initial_position_ = config_node["use_same_initial_position"].as<bool>();
  // init key_frame_manager
  key_frame_manager_ = std::make_shared<slt_common::LidarKeyFrameManager>(data_path);
  //
  registration_ = registration_factory_->create(config_node["registration"]);
  using VoxelFilter = slt_common::VoxelFilter;
  local_map_filter_ = std::make_shared<VoxelFilter>(config_node["local_map_filter"]);
  current_scan_filter_ = std::make_shared<VoxelFilter>(config_node["current_scan_filter"]);
  // get loop closure config:
  loop_closure_method_ = config_node["loop_closure_method"].as<std::string>();
  // create instance:
  scan_context_manager_ =
    std::make_shared<scan_context::ScanContextManager>(config_node[loop_closure_method_]);
  // print info
  std::cout << "point cloud registration:" << std::endl;
  registration_->print_info();
  std::cout << "local_map filter:" << std::endl;
  local_map_filter_->print_info();
  std::cout << "current_scan filter:" << std::endl;
  current_scan_filter_->print_info();
  return true;
}

bool LoopClosure::reset(const std::vector<slt_common::LidarFrame> & key_frames)
{
  key_frame_manager_->reset(key_frames);
  return true;
}

bool LoopClosure::detect(const slt_common::LidarFrame & current_frame)
{
  // load back current scan and add into scan context
  auto current_scan = key_frame_manager_->load_point_cloud(current_frame.index);
  scan_context_manager_->update(current_scan, current_frame.pose);
  // only perform loop closure detection for every skip_num key frames:
  if (++skip_cnt_ < loop_step_) {
    return false;
  }
  // detect by scan context
  if (!scan_context_manager_->detect_loop_closure()) {
    return false;
  }
  int matched_index = scan_context_manager_->get_frame_index();
  auto matched_frame = key_frame_manager_->get_key_frame(matched_index);
  // yaw change is unused, only output for debug
  std::cout << "detected doop closure: " << current_frame.index << "-" << matched_index << std::endl
            << "  scan context distance: " << scan_context_manager_->get_context_distance()
            << ", yaw change: " << scan_context_manager_->get_yaw_change() << std::endl;
  // check position difference:
  Eigen::Vector3d pos1 = current_frame.pose.block<3, 1>(0, 3);
  Eigen::Vector3d pos2 = matched_frame.pose.block<3, 1>(0, 3);
  double key_frame_distance = (pos1 - pos2).head<2>().norm();
  if (key_frame_distance > detect_area_) {
    std::cout << "  drop due to long distance: " << key_frame_distance << std::endl;
    return false;
  }
  // build local map
  size_t start = matched_index - extend_frame_num_;
  size_t end = matched_index + extend_frame_num_;
  if (matched_index < extend_frame_num_ || end >= key_frame_manager_->get_key_frame_count()) {
    std::cout << "  drop due to invalid extend frame." << std::endl;
    return false;
  }
  auto map = key_frame_manager_->get_local_map(start, end);
  // scan to map registration
  Eigen::Matrix4d current_frame_pose = current_frame.pose;
  if (use_same_initial_position_) {
    current_frame_pose.block<3, 1>(0, 3) = matched_frame.pose.block<3, 1>(0, 3);
  }
  registration_->set_target(current_scan_filter_->apply(map));
  registration_->match(current_scan_filter_->apply(current_scan), current_frame_pose);
  current_frame_pose = registration_->get_final_pose();
  double score = registration_->get_fitness_score();
  // check
  if (score > fitness_score_limit_) {
    std::cout << "  drop due to high registration score: " << score << std::endl;
    return false;
  }
  // current_loop_candidate
  current_loop_candidate_.index1 = current_frame.index;
  current_loop_candidate_.index2 = matched_frame.index;
  current_loop_candidate_.pose = current_frame_pose.inverse() * matched_frame.pose;
  valide_cnt_++;
  std::cout << "  it's valid, total loop closure pairs:" << valide_cnt_ << std::endl;
  // reset skip cnt
  skip_cnt_ = 0;
  return true;
}

slt_common::LoopCandidate & LoopClosure::get_loop_candidate()
{
  return current_loop_candidate_;
}

bool LoopClosure::save()
{
  std::filesystem::remove_all(scan_context_path_);
  if (!std::filesystem::create_directory(scan_context_path_)) {
    return false;
  }
  for (auto & key_frame : key_frame_manager_->get_key_frames()) {
    scan_context_manager_->update_key_frame_pose(key_frame.index, key_frame.pose);
  }
  return scan_context_manager_->save(scan_context_path_);
}

}  // namespace slt_lidar_mapping
