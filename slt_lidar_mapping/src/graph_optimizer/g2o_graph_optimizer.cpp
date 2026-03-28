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

#include "slt_lidar_mapping/graph_optimizer/g2o_graph_optimizer.hpp"

#include "slt_common/tic_toc.hpp"

namespace slt_lidar_mapping
{
G2oGraphOptimizer::G2oGraphOptimizer(const std::string & solver_type)
{
  graph_.reset(new g2o::SparseOptimizer());

  g2o::OptimizationAlgorithmFactory * solver_factory =
    g2o::OptimizationAlgorithmFactory::instance();
  // solver_factory->listSolvers(std::cout);
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm * solver = solver_factory->construct(solver_type, solver_property);
  graph_->setAlgorithm(solver);

  if (!graph_->solver()) {
    std::cerr << "Failed to create G2O optimizer!" << std::endl;
  }
  robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
}

bool G2oGraphOptimizer::optimize()
{
  static int optimize_cnt = 0;
  if (graph_->edges().size() < 1) {
    return false;
  }
  std::cout << std::endl
            << "------ Begin Iteration " << ++optimize_cnt << " of Backend Optimization -------"
            << std::endl;

  slt_common::TicToc optimize_time;
  graph_->initializeOptimization();
  graph_->computeInitialGuess();
  graph_->computeActiveErrors();
  graph_->setVerbose(false);

  double chi2 = graph_->chi2();
  int iterations = graph_->optimize(max_iterations_num_);

  std::cout << "Num. Vertices: " << graph_->vertices().size()
            << ", Num. Edges: " << graph_->edges().size() << std::endl
            << "Num. Iterations: " << iterations << "/" << max_iterations_num_ << std::endl
            << "Time Consumption: " << optimize_time.toc() << std::endl
            << "Cost Change: " << chi2 << "--->" << graph_->chi2() << std::endl
            << "------ Finish Backend Optimization -------" << std::endl
            << std::endl;

  return true;
}

bool G2oGraphOptimizer::get_optimized_pose(std::deque<Eigen::Matrix4f> & optimized_pose)
{
  optimized_pose.clear();
  int vertex_num = graph_->vertices().size();

  for (int i = 0; i < vertex_num; i++) {
    g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(graph_->vertex(i));
    Eigen::Isometry3d pose = v->estimate();
    optimized_pose.push_back(pose.matrix().cast<float>());
  }
  return true;
}

int G2oGraphOptimizer::get_node_num() {return graph_->vertices().size();}

void G2oGraphOptimizer::add_node(const Eigen::Isometry3d & pose, bool need_fix)
{
  g2o::VertexSE3 * vertex(new g2o::VertexSE3());

  vertex->setId(graph_->vertices().size());
  vertex->setEstimate(pose);
  if (need_fix) {
    vertex->setFixed(true);
  }

  graph_->addVertex(vertex);
}

void G2oGraphOptimizer::set_edge_robust_kernel(
  std::string robust_kernel_name, double robust_kernel_size)
{
  robust_kernel_name_ = robust_kernel_name;
  robust_kernel_size_ = robust_kernel_size;
  need_robust_kernel_ = true;
}

void G2oGraphOptimizer::add_relative_pose_edge(
  int vertex_index1, int vertex_index2, const Eigen::Isometry3d & relative_pose,
  const Eigen::VectorXd & noise)
{
  Eigen::MatrixXd information_matrix = calculate_relative_pose_edge_information_matrix(noise);

  g2o::VertexSE3 * v1 = dynamic_cast<g2o::VertexSE3 *>(graph_->vertex(vertex_index1));
  g2o::VertexSE3 * v2 = dynamic_cast<g2o::VertexSE3 *>(graph_->vertex(vertex_index2));

  g2o::EdgeSE3 * edge(new g2o::EdgeSE3());
  edge->setMeasurement(relative_pose);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  graph_->addEdge(edge);
  if (need_robust_kernel_) {
    add_robust_kernel(edge, robust_kernel_name_, robust_kernel_size_);
  }
}

Eigen::MatrixXd G2oGraphOptimizer::calculate_relative_pose_edge_information_matrix(
  const Eigen::VectorXd & noise)
{
  Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
  information_matrix = calculate_diag_matrix(noise);
  return information_matrix;
}

void G2oGraphOptimizer::add_robust_kernel(
  g2o::OptimizableGraph::Edge * edge, const std::string & kernel_type, double kernel_size)
{
  if (kernel_type == "NONE") {
    return;
  }

  g2o::RobustKernel * kernel = robust_kernel_factory_->construct(kernel_type);
  if (kernel == nullptr) {
    std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
    return;
  }

  kernel->setDelta(kernel_size);
  edge->setRobustKernel(kernel);
}

Eigen::MatrixXd G2oGraphOptimizer::calculate_diag_matrix(const Eigen::VectorXd & noise)
{
  Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
  for (int i = 0; i < noise.rows(); i++) {
    information_matrix(i, i) /= noise(i);
  }
  return information_matrix;
}

void G2oGraphOptimizer::add_prior_xyz_edge(
  int se3_vertex_index, const Eigen::Vector3d & xyz, const Eigen::VectorXd & noise)
{
  Eigen::MatrixXd information_matrix = calculate_diag_matrix(noise);
  g2o::VertexSE3 * v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_->vertex(se3_vertex_index));
  g2o::EdgeSE3PriorXYZ * edge(new g2o::EdgeSE3PriorXYZ());
  edge->setMeasurement(xyz);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph_->addEdge(edge);
}

void G2oGraphOptimizer::add_prior_quaternion_edge(
  int se3_vertex_index, const Eigen::Quaterniond & quat, const Eigen::VectorXd & noise)
{
  Eigen::MatrixXd information_matrix = calculate_prior_quaternion_edge_information_matrix(noise);
  g2o::VertexSE3 * v_se3 = dynamic_cast<g2o::VertexSE3 *>(graph_->vertex(se3_vertex_index));
  g2o::EdgeSE3PriorQuat * edge(new g2o::EdgeSE3PriorQuat());
  edge->setMeasurement(quat);
  edge->setInformation(information_matrix);
  edge->vertices()[0] = v_se3;
  graph_->addEdge(edge);
}

// TODO(all): 姿态观测的信息矩阵尚未添加
// 备注：各位使用时可只用位置观测，而不用姿态观测，影响不大
// 我自己在别的地方尝试过增加姿态观测，但效果反而变差，如果感兴趣，可自己编写此处的信息矩阵，并在后端优化中添加相应的边进行验证
Eigen::MatrixXd G2oGraphOptimizer::calculate_prior_quaternion_edge_information_matrix(
  const Eigen::VectorXd & /*noise*/)
{
  Eigen::MatrixXd information_matrix;
  return information_matrix;
}
}  // namespace slt_lidar_mapping
