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

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <memory>
#include <string>
#include <vector>
//
#include "slt_lio_mapping/graph_optimizer/g2o/edge_prvag_imu_pre_integration.hpp"
#include "slt_lio_mapping/graph_optimizer/g2o/edge_prvag_prior_pos.hpp"
#include "slt_lio_mapping/graph_optimizer/g2o/edge_prvag_relative_pose.hpp"
#include "slt_lio_mapping/graph_optimizer/g2o/vertex_prvag.hpp"
#include "slt_lio_mapping/graph_optimizer/graph_optimizer_interface.hpp"
#include "slt_imu_odometry/imu_pre_integration.hpp"

namespace g2o
{
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
}  // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

namespace slt_lio_mapping
{
class G2oGraphOptimizer : public GraphOptimizerInterface
{
public:
  explicit G2oGraphOptimizer(const YAML::Node & node);
  void set_gravity(Eigen::Vector3d gravity) {gravity_ = gravity;}
  //
  void add_vertex(const slt_common::ImuNavState & state, bool need_fix) override;
  void add_relative_pose_edge(
    int v0, int v1, const Eigen::Matrix4d & relative_pose, const Eigen::VectorXd & noise) override;
  void add_prior_position_edge(
    int v0, const Eigen::Vector3d & pos, const Eigen::Vector3d & noise) override;
  void add_imu_pre_integration_edge(
    int v0, int v1, const std::vector<slt_common::ImuData> & imus) override;
  // 优化
  bool optimize() override;
  // 输出数据
  int get_vertex_num() override;
  std::deque<slt_common::ImuNavState> get_optimized_vertices() override;

private:
  slt_common::ImuNavState create_nav_state(int vertex_id);
  void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size);
  Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);
  Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
  void AddRobustKernel(
    g2o::OptimizableGraph::Edge * edge, const std::string & kernel_type, double kernel_size);

private:
  g2o::RobustKernelFactory * robust_kernel_factory_;
  std::unique_ptr<g2o::SparseOptimizer> graph_;
  std::string robust_kernel_name_;
  double robust_kernel_size_;
  bool need_robust_kernel_ = false;
  int max_iterations_num_ = 512;
  //
  Eigen::Vector3d gravity_;
  std::shared_ptr<slt_imu_odometry::ImuPreIntegration> imu_pre_integration_;
};
}  // namespace slt_lio_mapping
