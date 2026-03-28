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
//
#include <deque>
#include <memory>
#include <string>
//
#include "slt_lidar_mapping/graph_optimizer/g2o/edge_se3_priorquat.hpp"
#include "slt_lidar_mapping/graph_optimizer/g2o/edge_se3_priorxyz.hpp"
#include "slt_lidar_mapping/graph_optimizer/graph_optimizer_interface.hpp"

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

// namespace g2o {
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
// G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
// } // namespace g2o

namespace slt_lidar_mapping
{
class G2oGraphOptimizer : public GraphOptimizerInterface
{
public:
  explicit G2oGraphOptimizer(const std::string & solver_type = "lm_var");
  // 添加节点、边
  void add_node(const Eigen::Isometry3d & pose, bool need_fix) override;
  void add_relative_pose_edge(
    int vertex_index1, int vertex_index2, const Eigen::Isometry3d & relative_pose,
    const Eigen::VectorXd & noise) override;
  void add_prior_xyz_edge(
    int se3_vertex_index, const Eigen::Vector3d & xyz, const Eigen::VectorXd & noise) override;
  void add_prior_quaternion_edge(
    int se3_vertex_index, const Eigen::Quaterniond & quat, const Eigen::VectorXd & noise) override;
  // 优化
  bool optimize() override;
  // 输出数据
  bool get_optimized_pose(std::deque<Eigen::Matrix4f> & optimized_pose) override;
  int get_node_num() override;
  // 设置鲁棒核
  void set_edge_robust_kernel(std::string robust_kernel_name, double robust_kernel_size);

private:
  Eigen::MatrixXd calculate_relative_pose_edge_information_matrix(const Eigen::VectorXd & noise);
  Eigen::MatrixXd calculate_prior_quaternion_edge_information_matrix(const Eigen::VectorXd & noise);
  Eigen::MatrixXd calculate_diag_matrix(const Eigen::VectorXd & noise);
  void add_robust_kernel(
    g2o::OptimizableGraph::Edge * edge, const std::string & kernel_type, double kernel_size);

private:
  g2o::RobustKernelFactory * robust_kernel_factory_;
  std::unique_ptr<g2o::SparseOptimizer> graph_;

  std::string robust_kernel_name_;
  double robust_kernel_size_;
  bool need_robust_kernel_ = false;
  int max_iterations_num_ = 512;
};
}  // namespace slt_lidar_mapping
