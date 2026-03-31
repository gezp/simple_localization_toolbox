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

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <string>
//
#include "slt_common/cloud_filter/voxel_filter.hpp"
#include "slt_common/point_cloud_registration/point_cloud_registration_factory.hpp"
#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/sensor_data/loop_candidate.hpp"
#include "slt_common/lidar_key_frame_manager.hpp"
#include "scan_context/scan_context_manager.hpp"

namespace slt_lidar_mapping
{
class LoopClosure
{
public:
  LoopClosure();
  bool init_config(const std::string & config_path, const std::string & data_path);
  bool reset(const std::vector<slt_common::LidarFrame> & key_frames);
  bool detect(const slt_common::LidarFrame & current_frame);
  slt_common::LoopCandidate & get_loop_candidate();
  bool save();

private:
  std::string data_path_ = "";
  std::string scan_context_path_ = "";
  std::string loop_closure_method_ = "";

  int extend_frame_num_ = 3;
  int loop_step_ = 10;
  int diff_num_ = 100;
  float detect_area_ = 10.0;
  float fitness_score_limit_ = 2.0;
  bool use_same_initial_position_{true};

  std::shared_ptr<slt_common::VoxelFilter> current_scan_filter_;
  std::shared_ptr<slt_common::VoxelFilter> local_map_filter_;
  std::shared_ptr<slt_common::PointCloudRegistrationInterface> registration_;
  std::shared_ptr<slt_common::PointCloudRegistrationFactory> registration_factory_;
  std::shared_ptr<slt_common::LidarKeyFrameManager> key_frame_manager_;
  std::shared_ptr<scan_context::ScanContextManager> scan_context_manager_;
  // data
  slt_common::LoopCandidate current_loop_candidate_;
  int skip_cnt_{0};
  size_t valide_cnt_{0};
};
}  // namespace slt_lidar_mapping
