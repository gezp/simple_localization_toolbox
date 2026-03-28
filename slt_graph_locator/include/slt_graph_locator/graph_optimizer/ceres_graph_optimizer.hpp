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

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_imu_odometry/imu_pre_integration.hpp"

namespace slt_graph_locator
{

struct ImuConfig
{
  double gyro_noise;
  double accel_noise;
  double gyro_bias_noise;
  double accel_bias_noise;
  Eigen::Vector3d gravity;
};

class CeresGraphOptimizer
{
public:
  static const int INDEX_P = 0;
  static const int INDEX_R = 3;
  static const int INDEX_V = 6;
  static const int INDEX_A = 9;
  static const int INDEX_G = 12;

  struct Vertex
  {
    size_t idx;
    double time;
    double state[15];
    bool fixed = false;
  };

  struct AbsolutePoseEdge
  {
    size_t vertex_idx;
    Eigen::VectorXd m;
    Eigen::MatrixXd I;
  };

  struct RelativePoseEdge
  {
    size_t vertex_idx0;
    size_t vertex_idx1;
    Eigen::VectorXd m;
    Eigen::MatrixXd I;
  };

  struct ImuPreIntegrationEdge
  {
    size_t vertex_idx0;
    size_t vertex_idx1;
    Eigen::VectorXd m;
    Eigen::MatrixXd I;
    double dt;
    Eigen::Vector3d gravity;
    Eigen::MatrixXd jacobian;
  };

  struct MarginalizationEdge
  {
    Vertex keep_vertex;
    Eigen::MatrixXd H;
    Eigen::VectorXd b;
  };

  explicit CeresGraphOptimizer(ImuConfig imu_config);
  ~CeresGraphOptimizer();

  // add vertex
  int add_vertex(const slt_common::ImuNavState & initial_state, bool need_fix);
  // add edge
  void add_relative_pose_edge(
    int vertex_idx0, int vertex_idx1, const Eigen::Matrix4d & pose, const Eigen::VectorXd & noise);
  void add_absolute_pose_edge(
    int vertex_idx, const Eigen::Matrix4d & pose, const Eigen::VectorXd & noise);
  void add_imu_pre_integration_edge(
    int vertex_idx0, int vertex_idx1, const std::vector<slt_common::ImuData> & imus);
  // do optimization
  bool optimize();
  bool marginalize(int drop_count = 1);
  // get result
  size_t get_vertex_count();
  slt_common::ImuNavState get_imu_nav_state(int idx = -1);

private:
  ceres::CostFunction * get_factor(const AbsolutePoseEdge & edge);
  ceres::CostFunction * get_factor(const RelativePoseEdge & edge);
  ceres::CostFunction * get_factor(const ImuPreIntegrationEdge & edge);
  Eigen::MatrixXd get_information_matrix(Eigen::VectorXd noise);
  slt_common::ImuNavState create_nav_state(size_t vertex_idx);

private:
  // config
  ImuConfig imu_config_;
  size_t keep_idx_{0};
  struct
  {
    std::unique_ptr<ceres::LossFunction> loss_function_ptr;
    ceres::Solver::Options options;
  } config_;
  // graph data
  std::vector<Vertex> vertices_;
  struct
  {
    std::deque<AbsolutePoseEdge> absolute_pose;
    std::deque<RelativePoseEdge> relative_pose;
    std::deque<ImuPreIntegrationEdge> imu_pre_integration;
    // at most one marginalization edge
    std::deque<MarginalizationEdge> marginalization;
  } edges_;
  // for imu pre integration
  std::shared_ptr<slt_imu_odometry::ImuPreIntegration> imu_pre_integration_;
};

}  // namespace slt_graph_locator
