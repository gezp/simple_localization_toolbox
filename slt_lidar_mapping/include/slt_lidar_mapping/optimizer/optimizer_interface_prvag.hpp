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
#include <deque>
#include <vector>

#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"

namespace slt_lidar_mapping
{

class GraphOptimizerInterfacePRVAG
{
public:
  virtual ~GraphOptimizerInterfacePRVAG() {}
  // 添加节点、边
  virtual void add_vertex(const slt_common::ImuNavState & state, bool need_fix) = 0;
  virtual void add_relative_pose_edge(
    int v0, int v1, const Eigen::Matrix4d & relative_pose, const Eigen::VectorXd & noise) = 0;
  virtual void add_prior_position_edge(
    int v0, const Eigen::Vector3d & pos, const Eigen::Vector3d & noise) = 0;
  virtual void add_imu_pre_integration_edge(
    int v0, int v1, const std::vector<slt_common::ImuData> & imus) = 0;
  // 优化
  virtual bool optimize() = 0;
  // 输出数据
  virtual std::deque<slt_common::ImuNavState> get_optimized_vertices() = 0;
  virtual int get_vertex_num() = 0;
};
}  // namespace slt_lidar_mapping
