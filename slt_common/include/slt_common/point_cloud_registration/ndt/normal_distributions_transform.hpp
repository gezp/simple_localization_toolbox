// Copyright 2026 Gezp (https://github.com/gezp).
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

// Ported from ndt_omp_ros2 (BSD License)
// Original: Point Cloud Library (PCL) - www.pointclouds.org

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

#include <cmath>
#include <memory>
#include <vector>

#include "slt_common/point_cloud_registration/ndt/voxel_grid_covariance.hpp"

namespace slt_common
{

enum NeighborSearchMethod
{
  KDTREE,
  DIRECT26,
  DIRECT7,
  DIRECT1
};

/** \brief A 3D Normal Distribution Transform registration implementation for point cloud data.
  * \note For more information please see
  * <b>Magnusson, M. (2009). The Three-Dimensional Normal-Distributions Transform —
  * an Efﬁcient Representation for Registration, Surface Analysis, and Loop Detection.
  * PhD thesis, Orebro University. Orebro Studies in Technology 36.</b>,
  * <b>More, J., and Thuente, D. (1994). Line Search Algorithm with Guaranteed Sufficient Decrease
  * In ACM Transactions on Mathematical Software.</b> and
  * Sun, W. and Yuan, Y, (2006) Optimization Theory and Methods: Nonlinear Programming. 89-100
  * \note Math refactored by Todor Stoyanov.
  * \author Brian Okorn (Space and Naval Warfare Systems Center Pacific)
  */
class NormalDistributionsTransform
{
public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using PointCloudPtr = PointCloud::Ptr;
  using PointCloudConstPtr = PointCloud::ConstPtr;

  using PointIndicesPtr = pcl::PointIndices::Ptr;
  using PointIndicesConstPtr = pcl::PointIndices::ConstPtr;

  /** \brief Typename of searchable voxel grid containing mean and covariance. */
  using TargetGrid = VoxelGridCovariance<pcl::PointXYZ>;
  /** \brief Typename of pointer to searchable voxel grid. */
  using TargetGridPtr = TargetGrid *;
  /** \brief Typename of const pointer to searchable voxel grid. */
  using TargetGridConstPtr = const TargetGrid *;
  /** \brief Typename of const pointer to searchable voxel grid leaf. */
  using TargetGridLeafConstPtr = const TargetGrid::Leaf *;

  /** \brief Constructor.
    * Sets \ref outlier_ratio_ to 0.35, \ref step_size_ to 0.05 and \ref resolution_ to 1.0
    */
  NormalDistributionsTransform();

  /** \brief Empty destructor */
  ~NormalDistributionsTransform() {}

  /** \brief Provide a pointer to the input source (e.g., the point cloud to be transformed).
    * \param[in] cloud the input point cloud source
    */
  void setInputSource(const PointCloudConstPtr & cloud) {input_ = cloud;}

  /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to).
    * \param[in] cloud the input point cloud target
    */
  void setInputTarget(const PointCloudConstPtr & cloud)
  {
    target_ = cloud;
    init();
  }

  /** \brief Set/change the voxel grid resolution.
    * \param[in] resolution side length of voxels
    */
  void setResolution(float resolution);

  /** \brief Get voxel grid resolution.
    * \return side length of voxels
    */
  float getResolution() const {return resolution_;}

  /** \brief Set/change the newton line search maximum step length.
    * \param[in] step_size maximum step length
    */
  void setStepSize(double step_size) {step_size_ = step_size;}

  /** \brief Get the newton line search maximum step length.
    * \return maximum step length
    */
  double getStepSize() const {return step_size_;}

  /** \brief Set the transformation epsilon (maximum allowable transformation epsilon).
    * \param[in] eps the transformation epsilon
    */
  void setTransformationEpsilon(double eps) {transformation_epsilon_ = eps;}

  /** \brief Get the transformation epsilon (maximum allowable transformation epsilon).
    * \return the transformation epsilon
    */
  double getTransformationEpsilon() const {return transformation_epsilon_;}

  /** \brief Set the maximum number of iterations.
    * \param[in] max_iter the maximum number of iterations
    */
  void setMaximumIterations(int max_iter) {max_iterations_ = max_iter;}

  /** \brief Get the maximum number of iterations.
    * \return the maximum number of iterations the internal optimization should run for
    */
  int getMaximumIterations() const {return max_iterations_;}

  /** \brief Set/change the point cloud outlier ratio.
    * \param[in] outlier_ratio outlier ratio
    */
  void setOulierRatio(double outlier_ratio) {outlier_ratio_ = outlier_ratio;}

  /** \brief Set the method to use for finding neighboring voxels.
    * \param[in] method the neighbor search method
    */
  void setNeighborhoodSearchMethod(NeighborSearchMethod method) {search_method_ = method;}

  /** \brief Align the input source to the target using an initial guess.
    * \param[in] guess the initial gross estimation of the transformation
    * \return true if alignment converged, false otherwise
    */
  bool align(const Eigen::Matrix4f & guess);

  /** \brief Get the final transformation matrix.
    * \return the final transformation matrix
    */
  Eigen::Matrix4f getFinalTransformation() const {return final_transformation_;}

  /** \brief Get the fitness score of the alignment.
    * \return the fitness score (negative log-likelihood per point)
    */
  double getFitnessScore() const {return calculateScore(*input_);}

  /** \brief Get the number of iterations required to calculate alignment.
    * \return final number of iterations
    */
  int getFinalNumIteration() const {return nr_iterations_;}

  /** \brief Convert 6 element transformation vector to affine transformation.
    * \param[in] x transformation vector of the form [x, y, z, roll, pitch, yaw]
    * \param[out] trans affine transform corresponding to given transformation vector
    */
  static void convertTransform(const Eigen::Matrix<double, 6, 1> & x, Eigen::Affine3f & trans)
  {
    trans = Eigen::Translation<float, 3>(
                static_cast<float>(x(0)), static_cast<float>(x(1)),
                static_cast<float>(x(2))) *
      Eigen::AngleAxis<float>(static_cast<float>(x(3)), Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxis<float>(static_cast<float>(x(4)), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxis<float>(static_cast<float>(x(5)), Eigen::Vector3f::UnitZ());
  }

  /** \brief Convert 6 element transformation vector to transformation matrix.
    * \param[in] x transformation vector of the form [x, y, z, roll, pitch, yaw]
    * \param[out] trans 4x4 transformation matrix corresponding to given transformation vector
    */
  static void convertTransform(const Eigen::Matrix<double, 6, 1> & x, Eigen::Matrix4f & trans)
  {
    Eigen::Affine3f _affine;
    convertTransform(x, _affine);
    trans = _affine.matrix();
  }

  /** \brief Calculate the negative log-likelihood score.
    * \param[in] cloud the transformed point cloud to evaluate
    * \return the score (lower is better)
    */
  double calculateScore(const PointCloud & cloud) const;

  NeighborSearchMethod search_method_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /** \brief Initiate covariance voxel structure. */
  void init();

  /** \brief Estimate the transformation.
    * \param[in] guess the initial gross estimation of the transformation
    */
  void computeTransformation(const Eigen::Matrix4f & guess);

  /** \brief Compute derivatives of probability function w.r.t. the transformation vector.
    * \param[out] score_gradient the gradient vector of the probability function w.r.t. the
    * transformation vector
    * \param[out] hessian the hessian matrix of the probability function w.r.t. the
    * transformation vector
    * \param[in] trans_cloud transformed point cloud
    * \param[in] p the current transform vector
    * \param[in] compute_hessian flag to calculate hessian, unnecessary for step calculation.
    * \return the score
    */
  double computeDerivatives(
    Eigen::Matrix<double, 6, 1> & score_gradient,
    Eigen::Matrix<double, 6, 6> & hessian,
    PointCloud & trans_cloud,
    Eigen::Matrix<double, 6, 1> & p,
    bool compute_hessian = true);

  /** \brief Compute individual point contributions to derivatives of probability function w.r.t.
    * the transformation vector.
    * \param[in,out] score_gradient the gradient vector
    * \param[in,out] hessian the hessian matrix
    * \param[in] point_gradient_ the point gradient
    * \param[in] point_hessian_ the point hessian
    * \param[in] x_trans transformed point minus mean of occupied covariance voxel
    * \param[in] c_inv covariance of occupied covariance voxel
    * \param[in] compute_hessian flag to calculate hessian
    * \return the score increment
    */
  double updateDerivatives(
    Eigen::Matrix<double, 6, 1> & score_gradient,
    Eigen::Matrix<double, 6, 6> & hessian,
    const Eigen::Matrix<float, 4, 6> & point_gradient_,
    const Eigen::Matrix<float, 24, 6> & point_hessian_,
    const Eigen::Vector3d & x_trans,
    const Eigen::Matrix3d & c_inv,
    bool compute_hessian = true) const;

  /** \brief Precompute angular components of derivatives.
    * \param[in] p the current transform vector
    * \param[in] compute_hessian flag to calculate hessian
    */
  void computeAngleDerivatives(Eigen::Matrix<double, 6, 1> & p, bool compute_hessian = true);

  /** \brief Compute point derivatives (float version).
    * \param[in] x point from the input cloud
    * \param[out] point_gradient_ the point gradient (float, 4x6)
    * \param[out] point_hessian_ the point hessian (float, 24x6)
    * \param[in] compute_hessian flag to calculate hessian
    */
  void computePointDerivatives(
    Eigen::Vector3d & x,
    Eigen::Matrix<float, 4, 6> & point_gradient_,
    Eigen::Matrix<float, 24, 6> & point_hessian_,
    bool compute_hessian = true) const;

  /** \brief Compute point derivatives (double version).
    * \param[in] x point from the input cloud
    * \param[out] point_gradient_ the point gradient (double, 3x6)
    * \param[out] point_hessian_ the point hessian (double, 18x6)
    * \param[in] compute_hessian flag to calculate hessian
    */
  void computePointDerivatives(
    Eigen::Vector3d & x,
    Eigen::Matrix<double, 3, 6> & point_gradient_,
    Eigen::Matrix<double, 18, 6> & point_hessian_,
    bool compute_hessian = true) const;

  /** \brief Compute hessian of probability function w.r.t. the transformation vector.
    * \param[out] hessian the hessian matrix
    * \param[in] trans_cloud transformed point cloud
    * \param[in] p the current transform vector
    */
  void computeHessian(
    Eigen::Matrix<double, 6, 6> & hessian,
    PointCloud & trans_cloud,
    Eigen::Matrix<double, 6, 1> & p);

  /** \brief Compute individual point contributions to hessian.
    * \param[in,out] hessian the hessian matrix
    * \param[in] point_gradient_ the point gradient
    * \param[in] point_hessian_ the point hessian
    * \param[in] x_trans transformed point minus mean of occupied covariance voxel
    * \param[in] c_inv covariance of occupied covariance voxel
    */
  void updateHessian(
    Eigen::Matrix<double, 6, 6> & hessian,
    const Eigen::Matrix<double, 3, 6> & point_gradient_,
    const Eigen::Matrix<double, 18, 6> & point_hessian_,
    const Eigen::Vector3d & x_trans,
    const Eigen::Matrix3d & c_inv) const;

  /** \brief Compute line search step length using More-Thuente method.
    * \param[in] x initial transformation vector
    * \param[in,out] step_dir descent direction (may be negated)
    * \param[in] step_init initial step length estimate
    * \param[in] step_max maximum step length
    * \param[in] step_min minimum step length
    * \param[out] score final score function value
    * \param[in,out] score_gradient gradient of score function
    * \param[out] hessian hessian of score function
    * \param[in,out] trans_cloud transformed point cloud
    * \return final step length
    */
  double computeStepLengthMT(
    const Eigen::Matrix<double, 6, 1> & x,
    Eigen::Matrix<double, 6, 1> & step_dir,
    double step_init,
    double step_max,
    double step_min,
    double & score,
    Eigen::Matrix<double, 6, 1> & score_gradient,
    Eigen::Matrix<double, 6, 6> & hessian,
    PointCloud & trans_cloud);

  /** \brief Update interval of possible step lengths for More-Thuente method.
    * \return if interval converges
    */
  bool updateIntervalMT(
    double & a_l, double & f_l, double & g_l,
    double & a_u, double & f_u, double & g_u,
    double a_t, double f_t, double g_t);

  /** \brief Select new trial value for More-Thuente method.
    * \return new trial value
    */
  double trialValueSelectionMT(
    double a_l, double f_l, double g_l,
    double a_u, double f_u, double g_u,
    double a_t, double f_t, double g_t);

  /** \brief Auxiliary function for More-Thuente interval determination.
    * \param[in] a the step length
    * \param[in] f_a function value at step length a
    * \param[in] f_0 initial function value
    * \param[in] g_0 initial function gradient
    * \param[in] mu sufficient decrease constant
    * \return sufficient decrease value
    */
  double auxilaryFunction_PsiMT(
    double a, double f_a, double f_0, double g_0, double mu = 1.e-4)
  {
    return  f_a - f_0 - mu * g_0 * a;
  }

  /** \brief Auxiliary function derivative for More-Thuente interval determination.
    * \param[in] g_a function gradient at step length a
    * \param[in] g_0 initial function gradient
    * \param[in] mu sufficient decrease constant
    * \return sufficient decrease derivative
    */
  double auxilaryFunction_dPsiMT(double g_a, double g_0, double mu = 1.e-4)
  {
    return  g_a - mu * g_0;
  }

private:
  /** \brief The voxel grid generated from target cloud containing point means and covariances. */
  TargetGrid target_cells_;

  /** \brief The side length of voxels. */
  float resolution_;

  /** \brief The maximum step length. */
  double step_size_;

  /** \brief Outlier ratio, Eq. 6.7 [Magnusson 2009]. */
  double outlier_ratio_;

  /** \brief Normalization constants, Eq. 6.8 [Magnusson 2009]. */
  double gauss_d1_, gauss_d2_, gauss_d3_;

  /** \brief Transformation probability score, Eq. 6.9 & 6.10 [Magnusson 2009]. */
  double trans_probability_;

  /** \brief Precomputed Angular Gradient
    * The precomputed angular derivatives for the jacobian of a transformation vector, Equation 6.19 [Magnusson 2009].
    */
  Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;

  Eigen::Matrix<float, 8, 4> j_ang_;

  /** \brief Precomputed Angular Hessian
    * The precomputed angular derivatives for the hessian of a transformation vector, Equation 6.19 [Magnusson 2009].
    */
  Eigen::Vector3d h_ang_a2_, h_ang_a3_,
    h_ang_b2_, h_ang_b3_,
    h_ang_c2_, h_ang_c3_,
    h_ang_d1_, h_ang_d2_, h_ang_d3_,
    h_ang_e1_, h_ang_e2_, h_ang_e3_,
    h_ang_f1_, h_ang_f2_, h_ang_f3_;

  Eigen::Matrix<float, 16, 4> h_ang_;

  // Members that were inherited from pcl::Registration:
  PointCloudConstPtr input_;
  PointCloudConstPtr target_;
  Eigen::Matrix4f final_transformation_;
  Eigen::Matrix4f transformation_;
  Eigen::Matrix4f previous_transformation_;
  int max_iterations_;
  int nr_iterations_;
  double transformation_epsilon_;
  bool converged_;
};

}  // namespace slt_common
