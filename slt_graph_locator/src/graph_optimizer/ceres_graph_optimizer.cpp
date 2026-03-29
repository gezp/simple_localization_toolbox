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

#include "slt_graph_locator/graph_optimizer/ceres_graph_optimizer.hpp"

#include <sophus/so3.hpp>

#include "slt_graph_locator/graph_optimizer/ceres/absolute_pose_factor.hpp"
#include "slt_graph_locator/graph_optimizer/ceres/relative_pose_factor.hpp"
#include "slt_graph_locator/graph_optimizer/ceres/imu_pre_integration_factor.hpp"
#include "slt_graph_locator/graph_optimizer/ceres/marginalization_factor.hpp"
#include "slt_graph_locator/graph_optimizer/ceres/prvag_manifold.hpp"

namespace slt_graph_locator
{

CeresGraphOptimizer::CeresGraphOptimizer(ImuConfig imu_config)
{
  imu_config_ = imu_config;
  // init imu pre-integration
  imu_pre_integration_ = std::make_shared<slt_imu_odometry::ImuPreIntegration>(
    imu_config_.accel_noise, imu_config_.gyro_noise, imu_config_.accel_bias_noise,
    imu_config_.gyro_bias_noise);
  // config optimizer:
  // a. loss function:
  config_.loss_function_ptr = std::make_unique<ceres::CauchyLoss>(1.0);
  // b. solver:
  config_.options.linear_solver_type = ceres::DENSE_SCHUR;
  // config_.options.use_explicit_schur_complement = true;
  config_.options.trust_region_strategy_type = ceres::DOGLEG;
  config_.options.num_threads = 1;
  config_.options.max_solver_time_in_seconds = 0.10;
  // config_.options.minimizer_progress_to_stdout = true;
  // clear data buffer:
  vertices_.clear();
  edges_.relative_pose.clear();
  edges_.absolute_pose.clear();
  edges_.imu_pre_integration.clear();
  edges_.marginalization.clear();
}

CeresGraphOptimizer::~CeresGraphOptimizer() {}

int CeresGraphOptimizer::add_vertex(
  const slt_common::ImuNavState & initial_state, bool need_fix)
{
  // check
  if (vertices_.size() > 0) {
    assert(initial_state.time > vertices_.back().time);
  }
  // create new vertex
  Vertex v;
  v.idx = vertices_.size();
  v.time = initial_state.time;
  v.fixed = need_fix;
  Eigen::Map<Eigen::Vector3d> pos(v.state + INDEX_P);
  Eigen::Map<Eigen::Vector3d> log_ori(v.state + INDEX_R);
  Eigen::Map<Eigen::Vector3d> vel(v.state + INDEX_V);
  Eigen::Map<Eigen::Vector3d> ba(v.state + INDEX_A);
  Eigen::Map<Eigen::Vector3d> bg(v.state + INDEX_G);
  pos = initial_state.position;
  Sophus::SO3d ori(Eigen::Quaterniond(initial_state.orientation));
  log_ori = ori.log();
  vel = initial_state.linear_velocity;
  ba = initial_state.accel_bias;
  bg = initial_state.gyro_bias;
  // add to graph
  vertices_.push_back(v);
  return v.idx;
}

void CeresGraphOptimizer::add_relative_pose_edge(
  int vertex_idx0, int vertex_idx1, const Eigen::Matrix4d & pose, const Eigen::VectorXd & noise)
{
  // create new edge
  RelativePoseEdge edge;
  edge.vertex_idx0 = vertex_idx0;
  edge.vertex_idx1 = vertex_idx1;
  edge.m = Eigen::VectorXd::Zero(6);
  edge.m.block<3, 1>(INDEX_P, 0) = pose.block<3, 1>(0, 3);
  edge.m.block<3, 1>(INDEX_R, 0) = Sophus::SO3d(Eigen::Quaterniond(pose.block<3, 3>(0, 0))).log();
  edge.I = get_information_matrix(noise);
  // add to graph
  edges_.relative_pose.push_back(edge);
}

void CeresGraphOptimizer::add_absolute_pose_edge(
  int vertex_idx, const Eigen::Matrix4d & pose, const Eigen::VectorXd & noise)
{
  // create new edge
  AbsolutePoseEdge edge;
  edge.vertex_idx = vertex_idx;
  edge.m = Eigen::VectorXd::Zero(6);
  edge.m.block<3, 1>(INDEX_P, 0) = pose.block<3, 1>(0, 3);
  edge.m.block<3, 1>(INDEX_R, 0) = Sophus::SO3d(Eigen::Quaterniond(pose.block<3, 3>(0, 0))).log();
  edge.I = get_information_matrix(noise);
  // add to graph
  edges_.absolute_pose.push_back(edge);
}

void CeresGraphOptimizer::add_imu_pre_integration_edge(
  int vertex_idx0, int vertex_idx1, const std::vector<slt_common::ImuData> & imus)
{
  // pre integration
  imu_pre_integration_->reset();
  for (auto & imu_data : imus) {
    imu_pre_integration_->integrate(imu_data);
  }
  // create new edge
  ImuPreIntegrationEdge edge;
  edge.vertex_idx0 = vertex_idx0;
  edge.vertex_idx1 = vertex_idx1;
  Eigen::Matrix<double, 15, 1> measurement = Eigen::Matrix<double, 15, 1>::Zero();
  measurement.block<3, 1>(0, 0) = imu_pre_integration_->get_alpha();
  Sophus::SO3d theta_ij(imu_pre_integration_->get_theta());
  measurement.block<3, 1>(3, 0) = theta_ij.log();
  measurement.block<3, 1>(6, 0) = imu_pre_integration_->get_beta();
  edge.m = measurement;
  edge.I = imu_pre_integration_->get_covariance().inverse();
  edge.dt = imu_pre_integration_->get_dt();
  edge.gravity = imu_config_.gravity;
  edge.jacobian = imu_pre_integration_->get_jacobian();
  // add to graph
  edges_.imu_pre_integration.push_back(edge);
}

ceres::CostFunction * CeresGraphOptimizer::get_factor(
  const CeresGraphOptimizer::AbsolutePoseEdge & edge)
{
  return new AbsolutePoseFactor(edge.m, edge.I);
}

ceres::CostFunction * CeresGraphOptimizer::get_factor(
  const CeresGraphOptimizer::RelativePoseEdge & edge)
{
  return new RelativePoseFactor(edge.m, edge.I);
}

ceres::CostFunction * CeresGraphOptimizer::get_factor(const ImuPreIntegrationEdge & edge)
{
  return new ImuPreIntegrationFactor(edge.m, edge.I, edge.jacobian, edge.gravity, edge.dt);
}

bool CeresGraphOptimizer::optimize()
{
  // create new sliding window optimization problem:
  ceres::Problem problem;
  // add parameter blocks:
  for (size_t i = keep_idx_; i < vertices_.size(); ++i) {
    auto & v = vertices_.at(i);
    ceres::Manifold * local_manifold = new PrvagManifold();
    problem.AddParameterBlock(v.state, 15);
    problem.SetManifold(v.state, local_manifold);
    if (v.fixed) {
      problem.SetParameterBlockConstant(v.state);
    }
  }
  // add residual blocks:
  // marginalization constraint
  if (!edges_.marginalization.empty()) {
    auto & edge = edges_.marginalization.back();
    MarginalizationFactor * factor_marginalization =
      new MarginalizationFactor(edge.H, edge.b, {edge.keep_vertex.state});
    auto & v = vertices_.at(edge.keep_vertex.idx);
    problem.AddResidualBlock(factor_marginalization, NULL, v.state);
  }
  // absolute pose constraint
  for (const auto & edge : edges_.absolute_pose) {
    auto & v = vertices_.at(edge.vertex_idx);
    ceres::CostFunction * factor_absolute_pose = get_factor(edge);
    problem.AddResidualBlock(factor_absolute_pose, NULL, v.state);
  }
  // relative pose constraint
  for (const auto & edge : edges_.relative_pose) {
    auto & v0 = vertices_.at(edge.vertex_idx0);
    auto & v1 = vertices_.at(edge.vertex_idx1);
    ceres::CostFunction * factor_relative_pose = get_factor(edge);
    problem.AddResidualBlock(factor_relative_pose, NULL, v0.state, v1.state);
  }
  // imu pre-integration constraint
  for (const auto & edge : edges_.imu_pre_integration) {
    auto & v0 = vertices_.at(edge.vertex_idx0);
    auto & v1 = vertices_.at(edge.vertex_idx1);
    ceres::CostFunction * factor_imu_pre_integration = get_factor(edge);
    problem.AddResidualBlock(factor_imu_pre_integration, NULL, v0.state, v1.state);
  }
  // solve:
  ceres::Solver::Summary summary;
  ceres::Solve(config_.options, &problem, &summary);
  // prompt:
  std::cout << "Cost Reduced: " << summary.initial_cost - summary.final_cost << std::endl
            << summary.BriefReport() << std::endl;
  return true;
}

bool CeresGraphOptimizer::marginalize(int drop_count)
{
  assert(drop_count > 0 && keep_idx_ + drop_count < vertices_.size());
  // create marginalization info:
  MarginalizationInfo marginalization_info;
  // parameter block info
  std::vector<double *> drop_parameter_blocks;
  std::vector<double *> keep_parameter_blocks;
  for (size_t i = keep_idx_; i < keep_idx_ + drop_count; i++) {
    drop_parameter_blocks.push_back(vertices_.at(i).state);
  }
  keep_idx_ = keep_idx_ + drop_count;
  keep_parameter_blocks.push_back(vertices_.at(keep_idx_).state);
  marginalization_info.set_parameter_blocks_info(drop_parameter_blocks, keep_parameter_blocks);
  // residual block info
  if (!edges_.marginalization.empty()) {
    auto & edge = edges_.marginalization.back();
    MarginalizationFactor * factor =
      new MarginalizationFactor(edge.H, edge.b, {edge.keep_vertex.state});
    marginalization_info.add_residual_block_info(factor, {vertices_[edge.keep_vertex.idx].state});
  }
  while (!edges_.absolute_pose.empty()) {
    auto & idx = edges_.absolute_pose.front().vertex_idx;
    if (idx >= keep_idx_) {
      break;
    }
    auto factor = get_factor(edges_.absolute_pose.front());
    marginalization_info.add_residual_block_info(factor, {vertices_[idx].state});
    edges_.absolute_pose.pop_front();
  }
  while (!edges_.relative_pose.empty()) {
    auto & idx0 = edges_.relative_pose.front().vertex_idx0;
    auto & idx1 = edges_.relative_pose.front().vertex_idx1;
    if (idx0 >= keep_idx_) {
      break;
    }
    auto factor = get_factor(edges_.relative_pose.front());
    marginalization_info.add_residual_block_info(
      factor, {vertices_[idx0].state, vertices_[idx1].state});
    edges_.relative_pose.pop_front();
  }
  while (!edges_.imu_pre_integration.empty()) {
    auto & idx0 = edges_.imu_pre_integration.front().vertex_idx0;
    auto & idx1 = edges_.imu_pre_integration.front().vertex_idx1;
    if (idx0 >= keep_idx_) {
      break;
    }
    auto factor = get_factor(edges_.imu_pre_integration.front());
    marginalization_info.add_residual_block_info(
      factor, {vertices_[idx0].state, vertices_[idx1].state});
    edges_.imu_pre_integration.pop_front();
  }
  // prompt:
  // std::cout << "drop edge count:" << marginalization_info.get_block_info_count() << std::endl;
  // clear last marginalization edge
  edges_.marginalization.clear();
  // create new marginalization edge
  MarginalizationEdge edge;
  if (marginalization_info.marginalize(edge.H, edge.b)) {
    edge.keep_vertex = vertices_.at(keep_idx_);
    // add to graph
    edges_.marginalization.push_back(edge);
  }
  return true;
}

size_t CeresGraphOptimizer::get_vertex_count() {return vertices_.size();}

slt_common::ImuNavState CeresGraphOptimizer::get_imu_nav_state(int idx)
{
  slt_common::ImuNavState state;
  if (idx >= 0) {
    state = create_nav_state(idx);
  } else {
    state = create_nav_state(vertices_.size() + idx);
  }
  return state;
}

Eigen::MatrixXd CeresGraphOptimizer::get_information_matrix(Eigen::VectorXd noise)
{
  Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
  for (int i = 0; i < noise.rows(); i++) {
    information_matrix(i, i) /= noise(i);
  }
  return information_matrix;
}

slt_common::ImuNavState CeresGraphOptimizer::create_nav_state(size_t vertex_idx)
{
  assert(vertex_idx < vertices_.size());
  // get vertext
  auto & v = vertices_.at(vertex_idx);
  Eigen::Map<const Eigen::Vector3d> pos(v.state + INDEX_P);
  Eigen::Map<const Eigen::Vector3d> log_ori(v.state + INDEX_R);
  Eigen::Map<const Eigen::Vector3d> vel(v.state + INDEX_V);
  Eigen::Map<const Eigen::Vector3d> ba(v.state + INDEX_A);
  Eigen::Map<const Eigen::Vector3d> bg(v.state + INDEX_G);
  // set state
  slt_common::ImuNavState state;
  state.time = v.time;
  state.position = pos;
  state.orientation = Sophus::SO3d::exp(log_ori).matrix();
  state.linear_velocity = vel;
  state.gravity = imu_config_.gravity;
  state.accel_bias = ba;
  state.gyro_bias = bg;
  return state;
}

}  // namespace slt_graph_locator
