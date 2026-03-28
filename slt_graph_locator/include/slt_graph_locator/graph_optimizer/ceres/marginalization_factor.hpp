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

#include <unordered_map>
#include <vector>

#include <sophus/so3.hpp>

namespace slt_graph_locator
{

struct ResidualBlockInfo
{
  ResidualBlockInfo(ceres::CostFunction * _factor, const std::vector<double *> & _parameter_blocks)
  : factor(_factor), parameter_blocks(_parameter_blocks)
  {
  }
  ~ResidualBlockInfo()
  {
    if (raw_jacobians) {
      delete[] raw_jacobians;
    }
  }
  void evaluate()
  {
    // init residual output:
    int D = static_cast<int>(factor->num_residuals());
    residuals.resize(D);
    // init jacobians output:
    std::vector<int> block_sizes = factor->parameter_block_sizes();
    int N = static_cast<int>(block_sizes.size());
    double ** raw_jacobians = new double *[N];
    jacobians.resize(N);
    // create raw pointer adaptor:
    for (int i = 0; i < N; i++) {
      jacobians[i].resize(D, block_sizes[i]);
      raw_jacobians[i] = jacobians[i].data();
    }
    factor->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);
  }

  ceres::CostFunction * factor;
  std::vector<double *> parameter_blocks;
  // evaluated result;
  double ** raw_jacobians = nullptr;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
  Eigen::VectorXd residuals;
};

class MarginalizationInfo
{
public:
  MarginalizationInfo() {}
  ~MarginalizationInfo() {}

  void set_parameter_blocks_info(
    const std::vector<double *> & marginalized_parameter_blocks,
    const std::vector<double *> & keep_parameter_blocks)
  {
    size_t pos = 0;
    for (auto & parameter_block : marginalized_parameter_blocks) {
      parameter_block_idx_[parameter_block] = pos;
      pos += 15;
    }
    marginalized_dim_ = pos;
    for (auto & parameter_block : keep_parameter_blocks) {
      parameter_block_idx_[parameter_block] = pos;
      pos += 15;
    }
    parameter_dim_ = pos;
  }

  void add_residual_block_info(
    ceres::CostFunction * factor, const std::vector<double *> & parameter_blocks)
  {
    residual_block_infos_.emplace_back(factor, parameter_blocks);
  }

  bool marginalize(Eigen::MatrixXd & H_keep, Eigen::VectorXd & b_keep)
  {
    // check edge associated with keep block
    size_t associated_edge_count = 0;
    for (auto & residual_block_info : residual_block_infos_) {
      for (auto & parameter_block : residual_block_info.parameter_blocks) {
        if (parameter_block_idx_[parameter_block] >= marginalized_dim_) {
          associated_edge_count++;
        }
      }
    }
    if (associated_edge_count == 0) {
      return false;
    }
    // dim
    size_t n = parameter_dim_;
    size_t m = marginalized_dim_;
    size_t r = n - m;
    // construct H, b
    Eigen::MatrixXd H(n, n);
    Eigen::VectorXd b(n);
    H.setZero();
    b.setZero();
    for (auto & residual_block_info : residual_block_infos_) {
      residual_block_info.evaluate();
      for (size_t i = 0; i < residual_block_info.parameter_blocks.size(); i++) {
        size_t block_i = parameter_block_idx_[residual_block_info.parameter_blocks[i]];
        Eigen::MatrixXd jacobian_i = residual_block_info.jacobians[i];
        for (size_t j = i; j < residual_block_info.parameter_blocks.size(); j++) {
          if (i == j) {
            H.block(block_i, block_i, 15, 15) += jacobian_i.transpose() * jacobian_i;
          } else {
            size_t block_j = parameter_block_idx_[residual_block_info.parameter_blocks[j]];
            Eigen::MatrixXd jacobian_j = residual_block_info.jacobians[j];
            H.block(block_i, block_j, 15, 15) += jacobian_i.transpose() * jacobian_j;
            H.block(block_j, block_i, 15, 15) = H.block(block_i, block_j, 15, 15).transpose();
          }
        }
        b.segment(block_i, 15) += jacobian_i.transpose() * residual_block_info.residuals;
      }
    }
    // schur complement
    Eigen::MatrixXd H_mm = H.block(0, 0, m, m);
    Eigen::MatrixXd H_mr = H.block(0, m, m, r);
    Eigen::MatrixXd H_rm = H.block(m, 0, r, m);
    Eigen::MatrixXd H_rr = H.block(m, m, r, r);
    Eigen::VectorXd b_m = b.segment(0, m);
    Eigen::VectorXd b_r = b.segment(m, r);
    H_keep = H_rr - H_rm * H_mm.inverse() * H_mr;
    b_keep = b_r - H_rm * H_mm.inverse() * b_m;
    return true;
  }

  size_t get_block_info_count() {return residual_block_infos_.size();}

private:
  size_t parameter_dim_;
  size_t marginalized_dim_;
  std::unordered_map<double *, size_t> parameter_block_idx_;
  std::vector<ResidualBlockInfo> residual_block_infos_;
};

class MarginalizationFactor : public ceres::CostFunction
{
public:
  MarginalizationFactor(
    const Eigen::MatrixXd & H, const Eigen::VectorXd & b, const std::vector<double *> & keep_blocks)
  {
    // linearized_jacobians_, linearized_residuals_
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H);
    Eigen::VectorXd S =
      Eigen::VectorXd((saes.eigenvalues().array() > 1.0e-5).select(saes.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd(
      (saes.eigenvalues().array() > 1.0e-5).select(saes.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
    linearized_jacobians_ = S_sqrt.asDiagonal() * saes.eigenvectors().transpose();
    linearized_residuals_ = S_inv_sqrt.asDiagonal() * saes.eigenvectors().transpose() * b;
    // keep blocks
    keep_blocks_ = keep_blocks;
    keep_dim_ = keep_blocks.size() * 15;
    // set cost function
    for (size_t i = 0; i < keep_blocks.size(); i++) {
      mutable_parameter_block_sizes()->push_back(15);
    }
    set_num_residuals(keep_dim_);
  }

  virtual bool Evaluate(
    double const * const * parameters, double * residuals, double ** jacobians) const
  {
    // compute residual
    Eigen::VectorXd dx(keep_dim_);
    for (size_t i = 0; i < keep_blocks_.size(); i++) {
      Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], 15);
      Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(keep_blocks_[i], 15);
      dx.segment(i * 15, 15) = x - x0;
    }
    Eigen::Map<Eigen::VectorXd>(residuals, keep_dim_) =
      linearized_residuals_ + linearized_jacobians_ * dx;
    // compute jacobian
    if (jacobians) {
      for (size_t i = 0; i < keep_blocks_.size(); i++) {
        if (jacobians[i]) {
          Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
          jacobian(jacobians[i], keep_dim_, 15);
          jacobian.setZero();
          jacobian = linearized_jacobians_.block(i * 15, 0, keep_dim_, 15);
        }
      }
    }
    return true;
  }

private:
  std::vector<double *> keep_blocks_;
  size_t keep_blocks_count_;
  size_t keep_dim_;
  Eigen::MatrixXd linearized_jacobians_;
  Eigen::VectorXd linearized_residuals_;
};

}  // namespace slt_graph_locator
