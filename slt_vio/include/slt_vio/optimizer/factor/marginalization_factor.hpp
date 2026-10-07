// Copyright 2026 Zhenpeng Ge (https://github.com/gezp).
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
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ceres/ceres.h"

#include "slt_common/math_utils.hpp"

namespace slt_vio
{
// one linearized factor of the marginalization, it owns its cost function
struct ResidualBlockInfo
{
  ResidualBlockInfo(
    ceres::CostFunction * cost_function, ceres::LossFunction * loss_function,
    const std::vector<double *> & parameter_blocks, const std::vector<int> & drop_set);
  ~ResidualBlockInfo();

  ResidualBlockInfo(const ResidualBlockInfo &) = delete;
  ResidualBlockInfo & operator=(const ResidualBlockInfo &) = delete;

  // evaluates the residuals and their jacobians, the loss function is absorbed into them
  void evaluate();

  ceres::CostFunction * cost_function;
  ceres::LossFunction * loss_function;
  std::vector<double *> parameter_blocks;
  // the index inside parameter_blocks of the blocks that are marginalized out
  std::vector<int> drop_set;

  double ** raw_jacobians{nullptr};
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
  Eigen::VectorXd residuals;
};

// the schur complement of the factors that involve the state leaving the window: a densely
// linearized prior on the remaining states, so the old frames' information is not thrown away
class MarginalizationInfo
{
public:
  ~MarginalizationInfo();

  MarginalizationInfo() = default;
  MarginalizationInfo(const MarginalizationInfo &) = delete;
  MarginalizationInfo & operator=(const MarginalizationInfo &) = delete;

  void add_residual_block_info(std::unique_ptr<ResidualBlockInfo> residual_block_info);
  // linearizes the factors at the values the blocks hold now, then eliminates the dropped ones
  void marginalize();
  // the parameter blocks left, at the addresses they entered with: a window block is never moved,
  // so the prior is handed on as it stands
  std::vector<double *> get_parameter_blocks();

  // the number of marginalized and of kept dimensions, the factor reads them
  int m{0};
  int n{0};
  std::vector<int> keep_block_size;
  std::vector<int> keep_block_idx;
  std::vector<double *> keep_block_data;
  Eigen::MatrixXd linearized_jacobians;
  Eigen::VectorXd linearized_residuals;

private:
  // the eigenvalues below it are treated as zero when the linearized system is decomposed
  static constexpr double kEigenvalueEps = 1e-8;
  std::vector<std::unique_ptr<ResidualBlockInfo>> factors_;
  // the global size of every parameter block that takes part
  std::unordered_map<intptr_t, int> parameter_block_size_;
  // the offset of every block inside the linearized system
  std::unordered_map<intptr_t, int> parameter_block_idx_;
  // the value of every block when the system was linearized
  std::unordered_map<intptr_t, std::vector<double>> parameter_block_data_;
};

// the prior the marginalization produced, it is added to the next problem as a single factor
class MarginalizationFactor : public ceres::CostFunction
{
  // the rotation of a state block, behind its position
  static constexpr int INDEX_R = 3;

public:
  explicit MarginalizationFactor(const MarginalizationInfo * marginalization_info);
  bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const override;

private:
  const MarginalizationInfo * marginalization_info_;
};

inline ResidualBlockInfo::ResidualBlockInfo(
  ceres::CostFunction * cost_function, ceres::LossFunction * loss_function,
  const std::vector<double *> & parameter_blocks, const std::vector<int> & drop_set)
: cost_function(cost_function),
  loss_function(loss_function),
  parameter_blocks(parameter_blocks),
  drop_set(drop_set)
{
}

inline ResidualBlockInfo::~ResidualBlockInfo()
{
  delete[] raw_jacobians;
  delete cost_function;
}

inline void ResidualBlockInfo::evaluate()
{
  residuals.resize(cost_function->num_residuals());
  const std::vector<int> block_sizes = cost_function->parameter_block_sizes();
  raw_jacobians = new double *[block_sizes.size()];
  jacobians.resize(block_sizes.size());
  for (size_t i = 0; i < block_sizes.size(); i++) {
    jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
    raw_jacobians[i] = jacobians[i].data();
  }
  cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

  if (!loss_function) {
    return;
  }
  // the robust kernel is absorbed into the jacobians, its gradient scaling is not a constant
  double rho[3];
  const double sq_norm = residuals.squaredNorm();
  loss_function->Evaluate(sq_norm, rho);
  const double sqrt_rho1 = std::sqrt(rho[1]);
  double residual_scaling;
  double alpha_sq_norm;
  if (sq_norm == 0.0 || rho[2] <= 0.0) {
    residual_scaling = sqrt_rho1;
    alpha_sq_norm = 0.0;
  } else {
    const double d = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
    const double alpha = 1.0 - std::sqrt(d);
    residual_scaling = sqrt_rho1 / (1 - alpha);
    alpha_sq_norm = alpha / sq_norm;
  }
  for (size_t i = 0; i < parameter_blocks.size(); i++) {
    jacobians[i] = sqrt_rho1 *
      (jacobians[i] - alpha_sq_norm * residuals * (residuals.transpose() * jacobians[i]));
  }
  residuals *= residual_scaling;
}

inline MarginalizationInfo::~MarginalizationInfo() = default;

inline void MarginalizationInfo::add_residual_block_info(
  std::unique_ptr<ResidualBlockInfo> residual_block_info)
{
  const std::vector<double *> & parameter_blocks = residual_block_info->parameter_blocks;
  const std::vector<int> parameter_block_sizes =
    residual_block_info->cost_function->parameter_block_sizes();
  for (size_t i = 0; i < parameter_blocks.size(); i++) {
    parameter_block_size_[reinterpret_cast<intptr_t>(parameter_blocks[i])] =
      parameter_block_sizes[i];
  }
  for (int index : residual_block_info->drop_set) {
    parameter_block_idx_[reinterpret_cast<intptr_t>(parameter_blocks[index])] = 0;
  }
  factors_.push_back(std::move(residual_block_info));
}

inline void MarginalizationInfo::marginalize()
{
  // every factor is linearized at the values the blocks hold now, and those values stay the
  // linearization point the prior is read at
  for (const auto & factor : factors_) {
    factor->evaluate();
    const std::vector<int> block_sizes = factor->cost_function->parameter_block_sizes();
    for (size_t i = 0; i < block_sizes.size(); i++) {
      const intptr_t addr = reinterpret_cast<intptr_t>(factor->parameter_blocks[i]);
      if (parameter_block_data_.find(addr) == parameter_block_data_.end()) {
        const double * data = factor->parameter_blocks[i];
        parameter_block_data_[addr] = std::vector<double>(data, data + block_sizes[i]);
      }
    }
  }

  int position = 0;
  for (auto & item : parameter_block_idx_) {
    item.second = position;
    position += parameter_block_size_[item.first];
  }
  m = position;
  for (const auto & item : parameter_block_size_) {
    if (parameter_block_idx_.find(item.first) == parameter_block_idx_.end()) {
      parameter_block_idx_[item.first] = position;
      position += item.second;
    }
  }
  n = position - m;

  // the information matrix of the linearized problem
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(position, position);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(position);
  for (const auto & factor : factors_) {
    for (size_t i = 0; i < factor->parameter_blocks.size(); i++) {
      const int idx_i =
        parameter_block_idx_[reinterpret_cast<intptr_t>(factor->parameter_blocks[i])];
      const int size_i =
        parameter_block_size_[reinterpret_cast<intptr_t>(factor->parameter_blocks[i])];
      const Eigen::MatrixXd jacobian_i = factor->jacobians[i].leftCols(size_i);
      for (size_t j = i; j < factor->parameter_blocks.size(); j++) {
        const int idx_j =
          parameter_block_idx_[reinterpret_cast<intptr_t>(factor->parameter_blocks[j])];
        const int size_j =
          parameter_block_size_[reinterpret_cast<intptr_t>(factor->parameter_blocks[j])];
        const Eigen::MatrixXd jacobian_j = factor->jacobians[j].leftCols(size_j);
        a.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
        if (i != j) {
          a.block(idx_j, idx_i, size_j, size_i) = a.block(idx_i, idx_j, size_i, size_j).transpose();
        }
      }
      b.segment(idx_i, size_i) += jacobian_i.transpose() * factor->residuals;
    }
  }

  // the schur complement, the pseudo inverse keeps the unobservable directions at zero
  const Eigen::MatrixXd a_mm = 0.5 * (a.block(0, 0, m, m) + a.block(0, 0, m, m).transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(a_mm);
  const Eigen::MatrixXd a_mm_inv = saes.eigenvectors() *
    Eigen::VectorXd(
    (saes.eigenvalues().array() > kEigenvalueEps).select(saes.eigenvalues().array().inverse(), 0))
    .asDiagonal() * saes.eigenvectors().transpose();
  const Eigen::VectorXd b_mm = b.segment(0, m);
  const Eigen::MatrixXd a_mr = a.block(0, m, m, n);
  const Eigen::MatrixXd a_rm = a.block(m, 0, n, m);
  const Eigen::MatrixXd a_rr = a.block(m, m, n, n);
  const Eigen::VectorXd b_rr = b.segment(m, n);
  const Eigen::MatrixXd h = a_rr - a_rm * a_mm_inv * a_mr;
  const Eigen::VectorXd rhs = b_rr - a_rm * a_mm_inv * b_mm;

  // the prior is stored in square root form, so the factor is a plain linear residual
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(h);
  const Eigen::VectorXd s = Eigen::VectorXd(
    (saes2.eigenvalues().array() > kEigenvalueEps).select(saes2.eigenvalues().array(), 0));
  const Eigen::VectorXd s_inv = Eigen::VectorXd(
    (saes2.eigenvalues().array() > kEigenvalueEps).select(saes2.eigenvalues().array().inverse(),
      0));
  linearized_jacobians = s.cwiseSqrt().asDiagonal() * saes2.eigenvectors().transpose();
  linearized_residuals = s_inv.cwiseSqrt().asDiagonal() * saes2.eigenvectors().transpose() * rhs;
}

inline std::vector<double *> MarginalizationInfo::get_parameter_blocks()
{
  std::vector<double *> keep_block_addr;
  keep_block_size.clear();
  keep_block_idx.clear();
  keep_block_data.clear();
  for (const auto & item : parameter_block_idx_) {
    if (item.second < m) {
      continue;
    }
    keep_block_size.push_back(parameter_block_size_[item.first]);
    keep_block_idx.push_back(item.second);
    keep_block_data.push_back(parameter_block_data_[item.first].data());
    // the key of a block is the address it entered the prior with, and it is still its own
    keep_block_addr.push_back(reinterpret_cast<double *>(item.first));
  }
  return keep_block_addr;
}

inline MarginalizationFactor::MarginalizationFactor(
  const MarginalizationInfo * marginalization_info)
: marginalization_info_(marginalization_info)
{
  for (int size : marginalization_info->keep_block_size) {
    mutable_parameter_block_sizes()->push_back(size);
  }
  set_num_residuals(marginalization_info->n);
}

inline bool MarginalizationFactor::Evaluate(
  double const * const * parameters, double * residuals, double ** jacobians) const
{
  const int n = marginalization_info_->n;
  const int m = marginalization_info_->m;
  Eigen::VectorXd dx(n);
  for (size_t i = 0; i < marginalization_info_->keep_block_size.size(); i++) {
    const int size = marginalization_info_->keep_block_size[i];
    const int idx = marginalization_info_->keep_block_idx[i] - m;
    const Eigen::Map<const Eigen::VectorXd> x(parameters[i], size);
    const Eigen::Map<const Eigen::VectorXd> x0(marginalization_info_->keep_block_data[i], size);
    if (size == 1) {
      dx.segment(idx, size) = x - x0;
      continue;
    }
    // the prior is linearized in the block's tangent space: its rotation is the right difference
    // the manifold retracts along, not the difference of the two log vectors
    dx.segment<3>(idx) = x.head<3>() - x0.head<3>();
    dx.segment<3>(idx + INDEX_R) =
      (Sophus::SO3d::exp(Eigen::Vector3d(x0.segment<3>(INDEX_R))).inverse() *
      Sophus::SO3d::exp(Eigen::Vector3d(x.segment<3>(INDEX_R)))).log();
    const int tail = size - (INDEX_R + 3);
    if (tail > 0) {
      dx.segment(idx + INDEX_R + 3, tail) = x.tail(tail) - x0.tail(tail);
    }
  }
  Eigen::Map<Eigen::VectorXd>(residuals, n) =
    marginalization_info_->linearized_residuals + marginalization_info_->linearized_jacobians * dx;
  if (!jacobians) {
    return true;
  }
  for (size_t i = 0; i < marginalization_info_->keep_block_size.size(); i++) {
    if (!jacobians[i]) {
      continue;
    }
    const int size = marginalization_info_->keep_block_size[i];
    const int idx = marginalization_info_->keep_block_idx[i] - m;
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(
      jacobians[i], n, size);
    jacobian.setZero();
    jacobian.leftCols(size) = marginalization_info_->linearized_jacobians.middleCols(idx, size);
  }
  return true;
}
}  // namespace slt_vio
