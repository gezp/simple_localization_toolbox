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

#include "slt_common/point_cloud_registration/ndt/normal_distributions_transform.hpp"

namespace slt_common
{

NormalDistributionsTransform::NormalDistributionsTransform()
: target_cells_(),
  resolution_(1.0f),
  step_size_(0.1),
  outlier_ratio_(0.55),
  gauss_d1_(),
  gauss_d2_(),
  gauss_d3_(),
  trans_probability_(),
  j_ang_a_(),
  j_ang_b_(),
  j_ang_c_(),
  j_ang_d_(),
  j_ang_e_(),
  j_ang_f_(),
  j_ang_g_(),
  j_ang_h_(),
  h_ang_a2_(),
  h_ang_a3_(),
  h_ang_b2_(),
  h_ang_b3_(),
  h_ang_c2_(),
  h_ang_c3_(),
  h_ang_d1_(),
  h_ang_d2_(),
  h_ang_d3_(),
  h_ang_e1_(),
  h_ang_e2_(),
  h_ang_e3_(),
  h_ang_f1_(),
  h_ang_f2_(),
  h_ang_f3_(),
  final_transformation_(Eigen::Matrix4f::Identity()),
  transformation_(Eigen::Matrix4f::Identity()),
  previous_transformation_(Eigen::Matrix4f::Identity()),
  max_iterations_(35),
  nr_iterations_(0),
  transformation_epsilon_(0.1),
  converged_(false),
  search_method_(DIRECT7)
{
  // Initializes the gaussian fitting parameters (eq. 6.8) [Magnusson 2009]
  double gauss_c1 = 10.0 * (1 - outlier_ratio_);
  double gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
  gauss_d3_ = -log(gauss_c2);
  gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3_;
  gauss_d2_ = -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3_) / gauss_d1_);
}

void NormalDistributionsTransform::setResolution(float resolution)
{
  if (resolution_ != resolution) {
    resolution_ = resolution;
    if (input_) {
      init();
    }
  }
}

void NormalDistributionsTransform::init()
{
  target_cells_.setLeafSize(resolution_, resolution_, resolution_);
  target_cells_.setInputCloud(target_);
  // Initiate voxel structure.
  target_cells_.filter(true);
}

bool NormalDistributionsTransform::align(const Eigen::Matrix4f & guess)
{
  PointCloud output;
  computeTransformation(guess);
  return converged_;
}

void NormalDistributionsTransform::computeTransformation(const Eigen::Matrix4f & guess)
{
  nr_iterations_ = 0;
  converged_ = false;

  // Initializes the gaussian fitting parameters (eq. 6.8) [Magnusson 2009]
  double gauss_c1 = 10.0 * (1 - outlier_ratio_);
  double gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
  gauss_d3_ = -log(gauss_c2);
  gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3_;
  gauss_d2_ = -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3_) / gauss_d1_);

  if (guess != Eigen::Matrix4f::Identity()) {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
  }

  Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
  eig_transformation.matrix() = final_transformation_;

  // Convert initial guess matrix to 6 element transformation vector
  Eigen::Matrix<double, 6, 1> p, delta_p, score_gradient;
  Eigen::Vector3f init_translation = eig_transformation.translation();
  Eigen::Vector3f init_rotation = eig_transformation.rotation().eulerAngles(0, 1, 2);
  p << init_translation(0), init_translation(1), init_translation(2),
    init_rotation(0), init_rotation(1), init_rotation(2);

  Eigen::Matrix<double, 6, 6> hessian;

  double score = 0;
  double delta_p_norm;

  // Apply guessed transformation prior to search for neighbours
  PointCloud output;
  pcl::transformPointCloud(*input_, output, guess);

  // Calculate derivatives of initial transform vector
  score = computeDerivatives(score_gradient, hessian, output, p);

  while (!converged_) {
    // Store previous transformation
    previous_transformation_ = transformation_;

    // Solve for descent direction using newton method, line 23 in Algorithm 2 [Magnusson 2009]
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(
      hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    // Negative for maximization as opposed to minimization
    delta_p = sv.solve(-score_gradient);

    // Calculate step length with guaranteed sufficient decrease [More, Thuente 1994]
    delta_p_norm = delta_p.norm();

    if (delta_p_norm == 0 || delta_p_norm != delta_p_norm) {
      trans_probability_ = score / static_cast<double>(input_->points.size());
      converged_ = delta_p_norm == delta_p_norm;
      return;
    }

    delta_p.normalize();
    delta_p_norm = computeStepLengthMT(
      p, delta_p, delta_p_norm, step_size_, transformation_epsilon_ / 2, score, score_gradient,
      hessian, output);
    delta_p *= delta_p_norm;

    transformation_ =
      (Eigen::Translation<float, 3>(
         static_cast<float>(delta_p(0)), static_cast<float>(delta_p(1)),
         static_cast<float>(delta_p(2))) *
      Eigen::AngleAxis<float>(static_cast<float>(delta_p(3)), Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxis<float>(static_cast<float>(delta_p(4)), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxis<float>(static_cast<float>(delta_p(5)), Eigen::Vector3f::UnitZ()))
      .matrix();

    p = p + delta_p;

    if (
      nr_iterations_ > max_iterations_ ||
      (nr_iterations_ && (std::fabs(delta_p_norm) < transformation_epsilon_)))
    {
      converged_ = true;
    }

    nr_iterations_++;
  }

  // Store transformation probability. The relative differences within each scan registration
  // are accurate but the normalization constants need to be modified for it to be globally accurate
  trans_probability_ = score / static_cast<double>(input_->points.size());

  // Update final transformation
  final_transformation_ =
    (Eigen::Translation<float, 3>(
       static_cast<float>(p(0)), static_cast<float>(p(1)), static_cast<float>(p(2))) *
    Eigen::AngleAxis<float>(static_cast<float>(p(3)), Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxis<float>(static_cast<float>(p(4)), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxis<float>(static_cast<float>(p(5)), Eigen::Vector3f::UnitZ()))
    .matrix();
}

double NormalDistributionsTransform::computeDerivatives(
  Eigen::Matrix<double, 6, 1> & score_gradient,
  Eigen::Matrix<double, 6, 6> & hessian,
  PointCloud & trans_cloud,
  Eigen::Matrix<double, 6, 1> & p,
  bool compute_hessian)
{
  score_gradient.setZero();
  hessian.setZero();
  double score = 0;

  // Precompute Angular Derivatives (eq. 6.19 and 6.21)[Magnusson 2009]
  computeAngleDerivatives(p);

  std::vector<TargetGridLeafConstPtr> neighborhood;
  std::vector<float> distances;

  // Update gradient and hessian for each point, line 17 in Algorithm 2 [Magnusson 2009]
  for (int idx = 0; idx < static_cast<int>(input_->points.size()); idx++) {
    // Original Point and Transformed Point
    pcl::PointXYZ x_pt, x_trans_pt;
    // Original Point and Transformed Point (for math)
    Eigen::Vector3d x, x_trans;
    // Occupied Voxel
    TargetGridLeafConstPtr cell;
    // Inverse Covariance of Occupied Voxel
    Eigen::Matrix3d c_inv;

    // Initialize Point Gradient and Hessian
    Eigen::Matrix<float, 4, 6> point_gradient_;
    Eigen::Matrix<float, 24, 6> point_hessian_;
    point_gradient_.setZero();
    point_gradient_.block<3, 3>(0, 0).setIdentity();
    point_hessian_.setZero();

    x_trans_pt = trans_cloud.points[idx];

    // Find neighbors (Radius search has been experimentally faster than direct neighbor checking.
    switch (search_method_) {
      case KDTREE:
        target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);
        break;
      case DIRECT26:
        target_cells_.getNeighborhoodAtPoint(x_trans_pt, neighborhood);
        break;
      default:
      case DIRECT7:
        target_cells_.getNeighborhoodAtPoint7(x_trans_pt, neighborhood);
        break;
      case DIRECT1:
        target_cells_.getNeighborhoodAtPoint1(x_trans_pt, neighborhood);
        break;
    }

    for (auto neighborhood_it = neighborhood.begin(); neighborhood_it != neighborhood.end();
      neighborhood_it++)
    {
      cell = *neighborhood_it;
      x_pt = input_->points[idx];
      x = Eigen::Vector3d(x_pt.x, x_pt.y, x_pt.z);

      x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

      // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson 2009]
      x_trans -= cell->getMean();
      // Uses precomputed covariance for speed.
      c_inv = cell->getInverseCov();

      // Compute derivative of transform function w.r.t. transform vector, J_E and H_E in
      // Equations 6.18 and 6.20 [Magnusson 2009]
      computePointDerivatives(x, point_gradient_, point_hessian_);
      // Update score, gradient and hessian, lines 19-21 in Algorithm 2, according to Equations
      // 6.10, 6.12 and 6.13, respectively [Magnusson 2009]
      score += updateDerivatives(
        score_gradient, hessian, point_gradient_, point_hessian_, x_trans, c_inv, compute_hessian);
    }
  }

  return  score;
}

void NormalDistributionsTransform::computeAngleDerivatives(
  Eigen::Matrix<double, 6, 1> & p, bool compute_hessian)
{
  // Simplified math for near 0 angles
  double cx, cy, cz, sx, sy, sz;
  if (fabs(p(3)) < 10e-5) {
    cx = 1.0;
    sx = 0.0;
  } else {
    cx = cos(p(3));
    sx = sin(p(3));
  }
  if (fabs(p(4)) < 10e-5) {
    cy = 1.0;
    sy = 0.0;
  } else {
    cy = cos(p(4));
    sy = sin(p(4));
  }

  if (fabs(p(5)) < 10e-5) {
    cz = 1.0;
    sz = 0.0;
  } else {
    cz = cos(p(5));
    sz = sin(p(5));
  }

  // Precomputed angular gradient components. Letters correspond to Equation 6.19 [Magnusson 2009]
  j_ang_a_ << (-sx * sz + cx * sy * cz), (-sx * cz - cx * sy * sz), (-cx * cy);
  j_ang_b_ << (cx * sz + sx * sy * cz), (cx * cz - sx * sy * sz), (-sx * cy);
  j_ang_c_ << (-sy * cz), sy * sz, cy;
  j_ang_d_ << sx * cy * cz, (-sx * cy * sz), sx * sy;
  j_ang_e_ << (-cx * cy * cz), cx * cy * sz, (-cx * sy);
  j_ang_f_ << (-cy * sz), (-cy * cz), 0;
  j_ang_g_ << (cx * cz - sx * sy * sz), (-cx * sz - sx * sy * cz), 0;
  j_ang_h_ << (sx * cz + cx * sy * sz), (cx * sy * cz - sx * sz), 0;

  j_ang_.setZero();
  j_ang_.row(0).noalias() =
    Eigen::Vector4f((-sx * sz + cx * sy * cz), (-sx * cz - cx * sy * sz), (-cx * cy), 0.0f);
  j_ang_.row(1).noalias() =
    Eigen::Vector4f((cx * sz + sx * sy * cz), (cx * cz - sx * sy * sz), (-sx * cy), 0.0f);
  j_ang_.row(2).noalias() = Eigen::Vector4f((-sy * cz), sy * sz, cy, 0.0f);
  j_ang_.row(3).noalias() = Eigen::Vector4f(sx * cy * cz, (-sx * cy * sz), sx * sy, 0.0f);
  j_ang_.row(4).noalias() = Eigen::Vector4f((-cx * cy * cz), cx * cy * sz, (-cx * sy), 0.0f);
  j_ang_.row(5).noalias() = Eigen::Vector4f((-cy * sz), (-cy * cz), 0, 0.0f);
  j_ang_.row(6).noalias() =
    Eigen::Vector4f((cx * cz - sx * sy * sz), (-cx * sz - sx * sy * cz), 0, 0.0f);
  j_ang_.row(7).noalias() =
    Eigen::Vector4f((sx * cz + cx * sy * sz), (cx * sy * cz - sx * sz), 0, 0.0f);

  if (compute_hessian) {
    // Precomputed angular hessian components. Letters correspond to Equation 6.21 and numbers
    // correspond to row index [Magnusson 2009]
    h_ang_a2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), sx * cy;
    h_ang_a3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), (-cx * cy);

    h_ang_b2_ << (cx * cy * cz), (-cx * cy * sz), (cx * sy);
    h_ang_b3_ << (sx * cy * cz), (-sx * cy * sz), (sx * sy);

    h_ang_c2_ << (-sx * cz - cx * sy * sz), (sx * sz - cx * sy * cz), 0;
    h_ang_c3_ << (cx * cz - sx * sy * sz), (-sx * sy * cz - cx * sz), 0;

    h_ang_d1_ << (-cy * cz), (cy * sz), (-sy);
    h_ang_d2_ << (-sx * sy * cz), (sx * sy * sz), (sx * cy);
    h_ang_d3_ << (cx * sy * cz), (-cx * sy * sz), (-cx * cy);

    h_ang_e1_ << (sy * sz), (sy * cz), 0;
    h_ang_e2_ << (-sx * cy * sz), (-sx * cy * cz), 0;
    h_ang_e3_ << (cx * cy * sz), (cx * cy * cz), 0;

    h_ang_f1_ << (-cy * cz), (cy * sz), 0;
    h_ang_f2_ << (-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), 0;
    h_ang_f3_ << (-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), 0;

    h_ang_.setZero();
    h_ang_.row(0).noalias() =
      Eigen::Vector4f((-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), sx * cy, 0.0f);
    h_ang_.row(1).noalias() =
      Eigen::Vector4f((-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), (-cx * cy), 0.0f);

    h_ang_.row(2).noalias() =
      Eigen::Vector4f((cx * cy * cz), (-cx * cy * sz), (cx * sy), 0.0f);
    h_ang_.row(3).noalias() =
      Eigen::Vector4f((sx * cy * cz), (-sx * cy * sz), (sx * sy), 0.0f);

    h_ang_.row(4).noalias() =
      Eigen::Vector4f((-sx * cz - cx * sy * sz), (sx * sz - cx * sy * cz), 0, 0.0f);
    h_ang_.row(5).noalias() =
      Eigen::Vector4f((cx * cz - sx * sy * sz), (-sx * sy * cz - cx * sz), 0, 0.0f);

    h_ang_.row(6).noalias() = Eigen::Vector4f((-cy * cz), (cy * sz), (sy), 0.0f);
    h_ang_.row(7).noalias() =
      Eigen::Vector4f((-sx * sy * cz), (sx * sy * sz), (sx * cy), 0.0f);
    h_ang_.row(8).noalias() =
      Eigen::Vector4f((cx * sy * cz), (-cx * sy * sz), (-cx * cy), 0.0f);

    h_ang_.row(9).noalias() = Eigen::Vector4f((sy * sz), (sy * cz), 0, 0.0f);
    h_ang_.row(10).noalias() =
      Eigen::Vector4f((-sx * cy * sz), (-sx * cy * cz), 0, 0.0f);
    h_ang_.row(11).noalias() = Eigen::Vector4f((cx * cy * sz), (cx * cy * cz), 0, 0.0f);

    h_ang_.row(12).noalias() = Eigen::Vector4f((-cy * cz), (cy * sz), 0, 0.0f);
    h_ang_.row(13).noalias() =
      Eigen::Vector4f((-cx * sz - sx * sy * cz), (-cx * cz + sx * sy * sz), 0, 0.0f);
    h_ang_.row(14).noalias() =
      Eigen::Vector4f((-sx * sz + cx * sy * cz), (-cx * sy * sz - sx * cz), 0, 0.0f);
  }
}

void NormalDistributionsTransform::computePointDerivatives(
  Eigen::Vector3d & x,
  Eigen::Matrix<float, 4, 6> & point_gradient_,
  Eigen::Matrix<float, 24, 6> & point_hessian_,
  bool compute_hessian) const
{
  Eigen::Vector4f x4(x[0], x[1], x[2], 0.0f);

  // Calculate first derivative of Transformation Equation 6.17 w.r.t. transform vector p.
  // Derivative w.r.t. ith element of transform vector corresponds to column i,
  // Equation 6.18 and 6.19 [Magnusson 2009]
  Eigen::Matrix<float, 8, 1> x_j_ang = j_ang_ * x4;

  point_gradient_(1, 3) = x_j_ang[0];
  point_gradient_(2, 3) = x_j_ang[1];
  point_gradient_(0, 4) = x_j_ang[2];
  point_gradient_(1, 4) = x_j_ang[3];
  point_gradient_(2, 4) = x_j_ang[4];
  point_gradient_(0, 5) = x_j_ang[5];
  point_gradient_(1, 5) = x_j_ang[6];
  point_gradient_(2, 5) = x_j_ang[7];

  if (compute_hessian) {
    Eigen::Matrix<float, 16, 1> x_h_ang = h_ang_ * x4;

    // Vectors from Equation 6.21 [Magnusson 2009]
    Eigen::Vector4f a(0, x_h_ang[0], x_h_ang[1], 0.0f);
    Eigen::Vector4f b(0, x_h_ang[2], x_h_ang[3], 0.0f);
    Eigen::Vector4f c(0, x_h_ang[4], x_h_ang[5], 0.0f);
    Eigen::Vector4f d(x_h_ang[6], x_h_ang[7], x_h_ang[8], 0.0f);
    Eigen::Vector4f e(x_h_ang[9], x_h_ang[10], x_h_ang[11], 0.0f);
    Eigen::Vector4f f(x_h_ang[12], x_h_ang[13], x_h_ang[14], 0.0f);

    // Calculate second derivative of Transformation Equation 6.17 w.r.t. transform vector p.
    // Derivative w.r.t. ith and jth elements of transform vector corresponds to the 3x1 block
    // matrix starting at (3i,j), Equation 6.20 and 6.21 [Magnusson 2009]
    point_hessian_.block<4, 1>((9 / 3) * 4, 3) = a;
    point_hessian_.block<4, 1>((12 / 3) * 4, 3) = b;
    point_hessian_.block<4, 1>((15 / 3) * 4, 3) = c;
    point_hessian_.block<4, 1>((9 / 3) * 4, 4) = b;
    point_hessian_.block<4, 1>((12 / 3) * 4, 4) = d;
    point_hessian_.block<4, 1>((15 / 3) * 4, 4) = e;
    point_hessian_.block<4, 1>((9 / 3) * 4, 5) = c;
    point_hessian_.block<4, 1>((12 / 3) * 4, 5) = e;
    point_hessian_.block<4, 1>((15 / 3) * 4, 5) = f;
  }
}

void NormalDistributionsTransform::computePointDerivatives(
  Eigen::Vector3d & x,
  Eigen::Matrix<double, 3, 6> & point_gradient_,
  Eigen::Matrix<double, 18, 6> & point_hessian_,
  bool compute_hessian) const
{
  // Calculate first derivative of Transformation Equation 6.17 w.r.t. transform vector p.
  // Derivative w.r.t. ith element of transform vector corresponds to column i,
  // Equation 6.18 and 6.19 [Magnusson 2009]
  point_gradient_(1, 3) = x.dot(j_ang_a_);
  point_gradient_(2, 3) = x.dot(j_ang_b_);
  point_gradient_(0, 4) = x.dot(j_ang_c_);
  point_gradient_(1, 4) = x.dot(j_ang_d_);
  point_gradient_(2, 4) = x.dot(j_ang_e_);
  point_gradient_(0, 5) = x.dot(j_ang_f_);
  point_gradient_(1, 5) = x.dot(j_ang_g_);
  point_gradient_(2, 5) = x.dot(j_ang_h_);

  if (compute_hessian) {
    // Vectors from Equation 6.21 [Magnusson 2009]
    Eigen::Vector3d a, b, c, d, e, f;

    a << 0, x.dot(h_ang_a2_), x.dot(h_ang_a3_);
    b << 0, x.dot(h_ang_b2_), x.dot(h_ang_b3_);
    c << 0, x.dot(h_ang_c2_), x.dot(h_ang_c3_);
    d << x.dot(h_ang_d1_), x.dot(h_ang_d2_), x.dot(h_ang_d3_);
    e << x.dot(h_ang_e1_), x.dot(h_ang_e2_), x.dot(h_ang_e3_);
    f << x.dot(h_ang_f1_), x.dot(h_ang_f2_), x.dot(h_ang_f3_);

    // Calculate second derivative of Transformation Equation 6.17 w.r.t. transform vector p.
    // Derivative w.r.t. ith and jth elements of transform vector corresponds to the 3x1 block
    // matrix starting at (3i,j), Equation 6.20 and 6.21 [Magnusson 2009]
    point_hessian_.block<3, 1>(9, 3) = a;
    point_hessian_.block<3, 1>(12, 3) = b;
    point_hessian_.block<3, 1>(15, 3) = c;
    point_hessian_.block<3, 1>(9, 4) = b;
    point_hessian_.block<3, 1>(12, 4) = d;
    point_hessian_.block<3, 1>(15, 4) = e;
    point_hessian_.block<3, 1>(9, 5) = c;
    point_hessian_.block<3, 1>(12, 5) = e;
    point_hessian_.block<3, 1>(15, 5) = f;
  }
}

double NormalDistributionsTransform::updateDerivatives(
  Eigen::Matrix<double, 6, 1> & score_gradient,
  Eigen::Matrix<double, 6, 6> & hessian,
  const Eigen::Matrix<float, 4, 6> & point_gradient4,
  const Eigen::Matrix<float, 24, 6> & point_hessian_,
  const Eigen::Vector3d & x_trans,
  const Eigen::Matrix3d & c_inv,
  bool compute_hessian) const
{
  Eigen::Matrix<float, 1, 4> x_trans4(x_trans[0], x_trans[1], x_trans[2], 0.0f);
  Eigen::Matrix4f c_inv4 = Eigen::Matrix4f::Zero();
  c_inv4.topLeftCorner(3, 3) = c_inv.cast<float>();

  float gauss_d2 = static_cast<float>(gauss_d2_);

  // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson 2009]
  float e_x_cov_x = exp(-gauss_d2 * x_trans4.dot(x_trans4 * c_inv4) * 0.5f);
  // Calculate probability of transformed points existence, Equation 6.9 [Magnusson 2009]
  float score_inc = -static_cast<float>(gauss_d1_) * e_x_cov_x;

  e_x_cov_x = gauss_d2 * e_x_cov_x;

  // Error checking for invalid values.
  if (e_x_cov_x > 1 || e_x_cov_x < 0 || e_x_cov_x != e_x_cov_x) {return  0;}

  // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
  e_x_cov_x *= static_cast<float>(gauss_d1_);

  Eigen::Matrix<float, 4, 6> c_inv4_x_point_gradient4 = c_inv4 * point_gradient4;
  Eigen::Matrix<float, 6, 1> x_trans4_dot_c_inv4_x_point_gradient4 =
    x_trans4 * c_inv4_x_point_gradient4;

  score_gradient.noalias() +=
    (e_x_cov_x * x_trans4_dot_c_inv4_x_point_gradient4).cast<double>();

  if (compute_hessian) {
    Eigen::Matrix<float, 1, 4> x_trans4_x_c_inv4 = x_trans4 * c_inv4;
    Eigen::Matrix<float, 6, 6> point_gradient4_colj_dot_c_inv4_x_point_gradient4_col_i =
      point_gradient4.transpose() * c_inv4_x_point_gradient4;
    Eigen::Matrix<float, 6, 1> x_trans4_dot_c_inv4_x_ext_point_hessian_4ij;

    for (int i = 0; i < 6; i++) {
      // Sigma_k^-1 d(T(x,p))/dpi, Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
      // Update gradient, Equation 6.12 [Magnusson 2009]
      x_trans4_dot_c_inv4_x_ext_point_hessian_4ij.noalias() =
        x_trans4_x_c_inv4 * point_hessian_.block<4, 6>(i * 4, 0);

      for (int j = 0; j < hessian.cols(); j++) {
        // Update hessian, Equation 6.13 [Magnusson 2009]
        hessian(i, j) +=
          e_x_cov_x *
          (-gauss_d2 * x_trans4_dot_c_inv4_x_point_gradient4(i) *
          x_trans4_dot_c_inv4_x_point_gradient4(j) +
          x_trans4_dot_c_inv4_x_ext_point_hessian_4ij(j) +
          point_gradient4_colj_dot_c_inv4_x_point_gradient4_col_i(j, i));
      }
    }
  }

  return  score_inc;
}

void NormalDistributionsTransform::computeHessian(
  Eigen::Matrix<double, 6, 6> & hessian,
  PointCloud & trans_cloud,
  Eigen::Matrix<double, 6, 1> &)
{
  // Original Point and Transformed Point
  pcl::PointXYZ x_pt, x_trans_pt;
  // Original Point and Transformed Point (for math)
  Eigen::Vector3d x, x_trans;
  // Occupied Voxel
  TargetGridLeafConstPtr cell;
  // Inverse Covariance of Occupied Voxel
  Eigen::Matrix3d c_inv;

  // Initialize Point Gradient and Hessian
  Eigen::Matrix<double, 3, 6> point_gradient_;
  Eigen::Matrix<double, 18, 6> point_hessian_;
  point_gradient_.setZero();
  point_gradient_.block<3, 3>(0, 0).setIdentity();
  point_hessian_.setZero();

  hessian.setZero();

  // Precompute angular derivatives (used after regular derivative calculation)

  // Update hessian for each point, line 17 in Algorithm 2 [Magnusson 2009]
  for (size_t idx = 0; idx < input_->points.size(); idx++) {
    x_trans_pt = trans_cloud.points[idx];

    // Find neighbors (Radius search has been experimentally faster than direct neighbor checking.
    std::vector<TargetGridLeafConstPtr> neighborhood;
    std::vector<float> distances;
    target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

    for (auto neighborhood_it = neighborhood.begin(); neighborhood_it != neighborhood.end();
      neighborhood_it++)
    {
      cell = *neighborhood_it;

      {
        x_pt = input_->points[idx];
        x = Eigen::Vector3d(x_pt.x, x_pt.y, x_pt.z);

        x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

        // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson 2009]
        x_trans -= cell->getMean();
        // Uses precomputed covariance for speed.
        c_inv = cell->getInverseCov();

        // Compute derivative of transform function w.r.t. transform vector, J_E and H_E in
        // Equations 6.18 and 6.20 [Magnusson 2009]
        computePointDerivatives(x, point_gradient_, point_hessian_);
        // Update hessian, lines 21 in Algorithm 2, according to Equations 6.10, 6.12 and 6.13,
        // respectively [Magnusson 2009]
        updateHessian(hessian, point_gradient_, point_hessian_, x_trans, c_inv);
      }
    }
  }
}

void NormalDistributionsTransform::updateHessian(
  Eigen::Matrix<double, 6, 6> & hessian,
  const Eigen::Matrix<double, 3, 6> & point_gradient_,
  const Eigen::Matrix<double, 18, 6> & point_hessian_,
  const Eigen::Vector3d & x_trans,
  const Eigen::Matrix3d & c_inv) const
{
  Eigen::Vector3d cov_dxd_pi;
  // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson 2009]
  double e_x_cov_x = gauss_d2_ * exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);

  // Error checking for invalid values.
  if (e_x_cov_x > 1 || e_x_cov_x < 0 || e_x_cov_x != e_x_cov_x) {return;}

  // Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
  e_x_cov_x *= gauss_d1_;

  for (int i = 0; i < 6; i++) {
    // Sigma_k^-1 d(T(x,p))/dpi, Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]
    cov_dxd_pi = c_inv * point_gradient_.col(i);

    for (int j = 0; j < hessian.cols(); j++) {
      // Update hessian, Equation 6.13 [Magnusson 2009]
      hessian(i, j) +=
        e_x_cov_x *
        (-gauss_d2_ * x_trans.dot(cov_dxd_pi) * x_trans.dot(c_inv * point_gradient_.col(j)) +
        x_trans.dot(c_inv * point_hessian_.block<3, 1>(3 * i, j)) +
        point_gradient_.col(j).dot(cov_dxd_pi));
    }
  }
}

bool NormalDistributionsTransform::updateIntervalMT(
  double & a_l, double & f_l, double & g_l,
  double & a_u, double & f_u, double & g_u,
  double a_t, double f_t, double g_t)
{
  // Case U1 [More, Thuente 1994]
  if (f_t > f_l) {
    a_u = a_t;
    f_u = f_t;
    g_u = g_t;
    return  false;
  } else if (g_t * (a_l - a_t) > 0) {
    a_l = a_t;
    f_l = f_t;
    g_l = g_t;
    return  false;
  } else if (g_t * (a_l - a_t) < 0) {
    a_u = a_l;
    f_u = f_l;
    g_u = g_l;

    a_l = a_t;
    f_l = f_t;
    g_l = g_t;
    return  false;
  }
  // Interval Converged
  return  true;
}

double NormalDistributionsTransform::trialValueSelectionMT(
  double a_l, double f_l, double g_l,
  double a_u, double f_u, double g_u,
  double a_t, double f_t, double g_t)
{
  // Case 1 in Trial Value Selection [More, Thuente 1994]
  if (f_t > f_l) {
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt(z * z - g_t * g_l);
    // Equation 2.4.56 [Sun, Yuan 2006]
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates f_l, f_t and g_l
    // Equation 2.4.2 [Sun, Yuan 2006]
    double a_q = a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

    if (std::fabs(a_c - a_l) < std::fabs(a_q - a_l)) {
      return  a_c;
    } else {
      return  0.5 * (a_q + a_c);
    }
  } else if (g_t * g_l < 0) {
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt(z * z - g_t * g_l);
    // Equation 2.4.56 [Sun, Yuan 2006]
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates f_l, g_l and g_t
    // Equation 2.4.5 [Sun, Yuan 2006]
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    if (std::fabs(a_c - a_t) >= std::fabs(a_s - a_t)) {
      return  a_c;
    } else {
      return  a_s;
    }
  } else if (std::fabs(g_t) <= std::fabs(g_l)) {
    // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
    double w = std::sqrt(z * z - g_t * g_l);
    double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

    // Calculate the minimizer of the quadratic that interpolates g_l and g_t
    // Equation 2.4.5 [Sun, Yuan 2006]
    double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

    double a_t_next;

    if (std::fabs(a_c - a_t) < std::fabs(a_s - a_t)) {
      a_t_next = a_c;
    } else {
      a_t_next = a_s;
    }

    if (a_t > a_l) {
      return  std::min(a_t + 0.66 * (a_u - a_t), a_t_next);
    } else {
      return  std::max(a_t + 0.66 * (a_u - a_t), a_t_next);
    }
  } else {
    // Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and g_t
    // Equation 2.4.52 [Sun, Yuan 2006]
    double z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
    double w = std::sqrt(z * z - g_t * g_u);
    // Equation 2.4.56 [Sun, Yuan 2006]
    return  a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w);
  }
}

double NormalDistributionsTransform::computeStepLengthMT(
  const Eigen::Matrix<double, 6, 1> & x,
  Eigen::Matrix<double, 6, 1> & step_dir,
  double step_init,
  double step_max,
  double step_min,
  double & score,
  Eigen::Matrix<double, 6, 1> & score_gradient,
  Eigen::Matrix<double, 6, 6> & hessian,
  PointCloud & trans_cloud)
{
  // Set the value of phi(0), Equation 1.3 [More, Thuente 1994]
  double phi_0 = -score;
  // Set the value of phi'(0), Equation 1.3 [More, Thuente 1994]
  double d_phi_0 = -(score_gradient.dot(step_dir));

  Eigen::Matrix<double, 6, 1> x_t;

  if (d_phi_0 >= 0) {
    // Not a descent direction
    if (d_phi_0 == 0) {
      return 0;
    } else {
      // Reverse step direction and calculate optimal step.
      d_phi_0 *= -1;
      step_dir *= -1;
    }
  }

  // The Search Algorithm for T(mu) [More, Thuente 1994]

  int max_step_iterations = 10;
  int step_iterations = 0;

  // Sufficient decrease constant, Equation 1.1 [More, Thuente 1994]
  double mu = 1.e-4;
  // Curvature condition constant, Equation 1.2 [More, Thuente 1994]
  double nu = 0.9;

  // Initial endpoints of Interval I,
  double a_l = 0, a_u = 0;

  // Auxiliary function psi is used until I is determined to be a closed interval,
  // Equation 2.1 [More, Thuente 1994]
  double f_l = auxilaryFunction_PsiMT(a_l, phi_0, phi_0, d_phi_0, mu);
  double g_l = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

  double f_u = auxilaryFunction_PsiMT(a_u, phi_0, phi_0, d_phi_0, mu);
  double g_u = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

  // Check used to allow More-Thuente step length calculation to be skipped by making
  // step_min == step_max
  bool interval_converged = (step_max - step_min) > 0, open_interval = true;

  double a_t = step_init;
  a_t = std::min(a_t, step_max);
  a_t = std::max(a_t, step_min);

  x_t = x + step_dir * a_t;

  final_transformation_ =
    (Eigen::Translation<float, 3>(
       static_cast<float>(x_t(0)), static_cast<float>(x_t(1)), static_cast<float>(x_t(2))) *
    Eigen::AngleAxis<float>(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxis<float>(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxis<float>(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ()))
    .matrix();

  // New transformed point cloud
  pcl::transformPointCloud(*input_, trans_cloud, final_transformation_);

  // Updates score, gradient and hessian. Hessian calculation is unnecessary but testing showed
  // that most step calculations use the initial step suggestion and recalculation the reusable
  // portions of the hessian would entail more computation time.
  score = computeDerivatives(score_gradient, hessian, trans_cloud, x_t, true);

  // Calculate phi(alpha_t)
  double phi_t = -score;
  // Calculate phi'(alpha_t)
  double d_phi_t = -(score_gradient.dot(step_dir));

  // Calculate psi(alpha_t)
  double psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
  // Calculate psi'(alpha_t)
  double d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

  // Iterate until max number of iterations, interval convergence or a value satisfies the
  // sufficient decrease, Equation 1.1, and curvature condition, Equation 1.2 [More, Thuente 1994]
  while (
    !interval_converged && step_iterations < max_step_iterations &&
    !(psi_t <= 0 /*Sufficient Decrease*/ && d_phi_t <= -nu * d_phi_0 /*Curvature Condition*/))
  {
    // Use auxiliary function if interval I is not closed
    if (open_interval) {
      a_t = trialValueSelectionMT(
        a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
    } else {
      a_t = trialValueSelectionMT(
        a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
    }

    a_t = std::min(a_t, step_max);
    a_t = std::max(a_t, step_min);

    x_t = x + step_dir * a_t;

    final_transformation_ =
      (Eigen::Translation<float, 3>(
         static_cast<float>(x_t(0)), static_cast<float>(x_t(1)), static_cast<float>(x_t(2))) *
      Eigen::AngleAxis<float>(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxis<float>(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxis<float>(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ()))
      .matrix();

    // New transformed point cloud
    // Done on final cloud to prevent wasted computation
    pcl::transformPointCloud(*input_, trans_cloud, final_transformation_);

    // Updates score, gradient. Values stored to prevent wasted computation.
    score = computeDerivatives(score_gradient, hessian, trans_cloud, x_t, false);

    // Calculate phi(alpha_t+)
    phi_t = -score;
    // Calculate phi'(alpha_t+)
    d_phi_t = -(score_gradient.dot(step_dir));

    // Calculate psi(alpha_t+)
    psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
    // Calculate psi'(alpha_t+)
    d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

    // Check if I is now a closed interval
    if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
      open_interval = false;

      // Converts f_l and g_l from psi to phi
      f_l = f_l + phi_0 - mu * d_phi_0 * a_l;
      g_l = g_l + mu * d_phi_0;

      // Converts f_u and g_u from psi to phi
      f_u = f_u + phi_0 - mu * d_phi_0 * a_u;
      g_u = g_u + mu * d_phi_0;
    }

    if (open_interval) {
      // Update interval end points using Updating Algorithm [More, Thuente 1994]
      interval_converged =
        updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
    } else {
      // Update interval end points using Modified Updating Algorithm [More, Thuente 1994]
      interval_converged =
        updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
    }

    step_iterations++;
  }

  // If inner loop was run then hessian needs to be calculated.
  // Hessian is unnecessary for step length determination but gradients are required
  // so derivative and transform data is stored for the next iteration.
  if (step_iterations) {computeHessian(hessian, trans_cloud, x_t);}

  return  a_t;
}

double NormalDistributionsTransform::calculateScore(const PointCloud & trans_cloud) const
{
  double score = 0;

  for (int idx = 0; idx < static_cast<int>(trans_cloud.points.size()); idx++) {
    pcl::PointXYZ x_trans_pt = trans_cloud.points[idx];

    // Find neighbors (Radius search has been experimentally faster than direct neighbor checking.
    std::vector<TargetGridLeafConstPtr> neighborhood;
    std::vector<float> distances;
    target_cells_.radiusSearch(x_trans_pt, resolution_, neighborhood, distances);

    for (auto neighborhood_it = neighborhood.begin(); neighborhood_it != neighborhood.end();
      neighborhood_it++)
    {
      TargetGridLeafConstPtr cell = *neighborhood_it;

      Eigen::Vector3d x_trans = Eigen::Vector3d(x_trans_pt.x, x_trans_pt.y, x_trans_pt.z);

      // Denorm point, x_k' in Equations 6.12 and 6.13 [Magnusson 2009]
      x_trans -= cell->getMean();
      // Uses precomputed covariance for speed.
      Eigen::Matrix3d c_inv = cell->getInverseCov();

      // e^(-d_2/2 * (x_k - mu_k)^T Sigma_k^-1 (x_k - mu_k)) Equation 6.9 [Magnusson 2009]
      double e_x_cov_x = exp(-gauss_d2_ * x_trans.dot(c_inv * x_trans) / 2);
      // Calculate probability of transformed points existence, Equation 6.9 [Magnusson 2009]
      double score_inc = -gauss_d1_ * e_x_cov_x - gauss_d3_;

      score += score_inc / neighborhood.size();
    }
  }
  return (score) / static_cast<double>(trans_cloud.size());
}

}  // namespace slt_common
