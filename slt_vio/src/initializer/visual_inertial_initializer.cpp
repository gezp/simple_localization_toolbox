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

#include "slt_vio/initializer/visual_inertial_initializer.hpp"

#include <cassert>
#include <cmath>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "slt_vio/initializer/global_sfm.hpp"

namespace slt_vio
{
VisualInertialInitializer::VisualInertialInitializer(const YAML::Node & config)
{
  window_size_ = config["window_size"].as<int>(window_size_);
  focal_length_ = config["focal_length"].as<double>(focal_length_);
  // gravity magnitude, the alignment only refines its direction
  g_norm_ = config["g_norm"].as<double>(g_norm_);
  elapsed_time_statistics_.set_enable(config["enable_elapsed_time_statistics"].as<bool>(false));
  elapsed_time_statistics_.set_title("VisualInertialInitializer");
  reset();
}

void VisualInertialInitializer::set_extrinsic(const Eigen::Matrix4d & T_imu_camera)
{
  T_imu_camera_ = T_imu_camera;
}

void VisualInertialInitializer::reset()
{
  frames_.clear();
  imu_edges_.clear();

  gyro_bias_.setZero();
  gravity_ = Eigen::Vector3d(0, 0, -g_norm_);
  last_attempt_time_ = 0.0;
  initialized_ = false;
}

std::string VisualInertialInitializer::error_message()
{
  std::string message = message_;
  message_.clear();
  return message;
}

bool VisualInertialInitializer::try_init()
{
  if (initialized_) {
    return true;
  }

  // a window over the steady size holds one frame no attempt has read, the newest
  if (frames_.size() < static_cast<size_t>(window_size_)) {
    return false;
  }

  // the sfm runs only once the gap has passed
  if (frames_.back().time - last_attempt_time_ <= kInitializationGap) {
    return false;
  }
  last_attempt_time_ = frames_.back().time;

  // warned about, not acted on: the sfm decides whether the window is worth initializing
  if (!check_imu_observibility()) {
    message_ = "imu excitation not enough, the gravity alignment is unreliable";
  }

  elapsed_time_statistics_.tic("structure_from_motion");
  const bool sfm_success = structure_from_motion();
  elapsed_time_statistics_.toc("structure_from_motion");
  if (!sfm_success) {
    return false;
  }

  // solve the bias increment the edges disagree with the sfm about first: they are linearized where
  // last reintegrated, so the solve is added to that bias and they are reintegrated, not corrected
  elapsed_time_statistics_.tic("solve_gyroscope_bias");
  gyro_bias_ += solve_gyroscope_bias();
  elapsed_time_statistics_.toc("solve_gyroscope_bias");
  for (auto & edge : imu_edges_) {
    if (edge.pre_integration) {
      edge.pre_integration->set_bias(Eigen::Vector3d::Zero(), gyro_bias_);
      edge.pre_integration->reintegrate();
    }
  }
  // the gravity, velocity and scale are returned through the arguments
  std::vector<Eigen::Vector3d> velocities;
  Eigen::Vector3d gravity;
  double s = 0.0;
  elapsed_time_statistics_.tic("linear_alignment");
  const bool aligned = linear_alignment(velocities, gravity, s);
  elapsed_time_statistics_.toc("linear_alignment");
  if (!aligned) {
    message_ = "solving the gravity and the scale failed";
    return false;
  }

  const Eigen::Matrix3d R_bc = T_imu_camera_.block<3, 3>(0, 0);
  const Eigen::Vector3d t_bc = T_imu_camera_.block<3, 1>(0, 3);
  const Eigen::Vector3d t_c0 = frames_.front().T_c.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_b0 = frames_.front().T_c.block<3, 3>(0, 0) * R_bc.transpose();
  // the world is first turned so its z follows gravity, the yaw that turn leaves being arbitrary
  Eigen::Matrix3d r0 =
    Eigen::Quaterniond::FromTwoVectors(gravity.normalized(), Eigen::Vector3d{0, 0, -1.0})
    .toRotationMatrix();
  double yaw = slt_common::R_to_ypr(r0).x();
  r0 = slt_common::ypr_to_R(Eigen::Vector3d{-yaw, 0, 0}) * r0;
  // and then so its yaw follows the first camera, which leaves the first pose at the origin
  yaw = slt_common::R_to_ypr(r0 * R_b0).x();
  r0 = slt_common::ypr_to_R(Eigen::Vector3d(-yaw, 0, 0)) * r0;
  gravity_ = r0 * gravity;
  for (int i = 0; i < static_cast<int>(frames_.size()); i++) {
    auto & frame = frames_[i];
    const Eigen::Matrix3d R_i = frame.T_c.block<3, 3>(0, 0) * R_bc.transpose();
    const Eigen::Vector3d t_i = frame.T_c.block<3, 1>(0, 3);
    frame.state.orientation = r0 * R_i;
    frame.state.position = r0 * (s * t_i - R_i * t_bc - (s * t_c0 - R_b0 * t_bc));
    frame.state.linear_velocity = r0 * R_i * velocities[i];
    frame.state.gyro_bias = gyro_bias_;
    frame.state.gravity = gravity_;
  }
  // the scale the structure from motion was aligned with and the gravity the alignment settled on
  message_ = "initialization finished, sfm scale " + std::to_string(s) +
    ", gravity " + std::to_string(gravity_.norm());
  initialized_ = true;
  elapsed_time_statistics_.print_all_info();
  return true;
}

void VisualInertialInitializer::add_frame(
  const FeatureFrame & feature_frame,
  const slt_imu_odometry::ImuPreIntegrationPtr & pre_integration, bool is_key_frame)
{
  ImageFrame frame(feature_frame.features, feature_frame.time);
  frame.frame_id = feature_frame.frame_id;
  frame.state.time = feature_frame.time;
  frame.state.gravity = gravity_;
  frame.is_key_frame = is_key_frame;
  // the frame that opens a window closes no edge, its interval dropped; the others open the edge
  // ending at them, on that interval or on the dropped frame's with this one integrated onto it
  slt_imu_odometry::ImuPreIntegrationPtr edge_pre_integration = pre_integration;

  if (frames_.size() >= static_cast<size_t>(window_size_)) {
    if (!frames_.back().is_key_frame) {
      // merge last pre_integration and current pre_integration
      auto last_pre_integration = imu_edges_.back().pre_integration;
      for (const auto & imu : pre_integration->get_imu_data_buffer()) {
        last_pre_integration->integrate(imu);
      }
      edge_pre_integration = last_pre_integration;
      imu_edges_.pop_back();
      frames_.pop_back();
    } else {
      imu_edges_.pop_front();
      frames_.pop_front();
    }
  }

  if (!frames_.empty()) {
    ImuEdge edge;
    edge.from = frames_.back().frame_id;
    edge.to = feature_frame.frame_id;
    edge.pre_integration = edge_pre_integration;
    imu_edges_.push_back(std::move(edge));
  }
  frames_.push_back(std::move(frame));
}

bool VisualInertialInitializer::check_imu_observibility()
{
  // the gravity the edges measure has to vary for the alignment to mean anything
  Eigen::Vector3d sum_g = Eigen::Vector3d::Zero();
  int count = 0;
  for (const auto & interval : imu_edges_) {
    sum_g += interval.pre_integration->get_beta() / interval.pre_integration->get_delta_time();
    count++;
  }
  // nothing to measure is not a lack of excitation
  if (count == 0) {
    return true;
  }
  const Eigen::Vector3d aver_g = sum_g / count;
  double var = 0.0;
  for (const auto & interval : imu_edges_) {
    const Eigen::Vector3d tmp_g =
      interval.pre_integration->get_beta() / interval.pre_integration->get_delta_time();
    var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
  }
  return std::sqrt(var / count) >= 0.25;
}

bool VisualInertialInitializer::structure_from_motion()
{
  // the window as the sfm reads it: one table per frame, keyed by feature id, in arrival order
  std::vector<std::map<int, Eigen::Vector2d>> sfm_frames;
  sfm_frames.reserve(frames_.size());
  for (const auto & frame : frames_) {
    std::map<int, Eigen::Vector2d> sfm_frame;
    for (const auto & id_pts : frame.features) {
      sfm_frame[id_pts.first] = id_pts.second.xy;
    }
    sfm_frames.push_back(sfm_frame);
  }

  // the sfm reads the normalized image, the parallax it starts from is given in it
  GlobalSfm sfm(kInitParallaxPx / focal_length_);
  std::vector<Eigen::Matrix4d> frame_poses;
  if (!sfm.reconstruct(sfm_frames, frame_poses)) {
    if (sfm.status() == SfmStatus::kNoRelativePose) {
      message_ = "not enough features or parallax, move the device around";
      return false;
    }
    // the oldest frame leaves, whatever the newest one is
    frames_.back().is_key_frame = true;
    message_ = "global sfm failed";
    return false;
  }
  // the sfm settles the pose of every frame of the window
  for (int i = 0; i < static_cast<int>(frames_.size()); i++) {
    frames_[i].T_c = frame_poses[i];
  }
  return true;
}

Eigen::Vector3d VisualInertialInitializer::solve_gyroscope_bias()
{
  // the rotation the preintegration predicts must match the sfm's, the mismatch is linear in bg
  const Eigen::Matrix3d R_bc = T_imu_camera_.block<3, 3>(0, 0);
  Eigen::Matrix3d a = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();
  // the edge between two frames is the one the second closed, the oldest opening none: the queue
  // holds one fewer than the window holds frames
  assert(imu_edges_.size() + 1 == frames_.size());
  for (size_t i = 0; i < imu_edges_.size(); i++) {
    const auto & pre_integration = imu_edges_[i].pre_integration;
    const Eigen::Matrix3d R_i = frames_[i].T_c.block<3, 3>(0, 0) * R_bc.transpose();
    const Eigen::Matrix3d R_j = frames_[i + 1].T_c.block<3, 3>(0, 0) * R_bc.transpose();
    // the theta row of the preintegration jacobian against the gyroscope bias
    const Eigen::Matrix3d tmp_a = pre_integration->get_jacobian().block<3, 3>(3, 12);
    // the mismatch in the tangent space the jacobian is written in: the so3 log of the rotation
    // the preintegration is off by
    const Eigen::Vector3d tmp_b =
      Sophus::SO3d(pre_integration->get_theta().transpose() * R_i.transpose() * R_j).log();
    a += tmp_a.transpose() * tmp_a;
    b += tmp_a.transpose() * tmp_b;
  }
  // the increment the edges are short by from the bias they carry: the caller adds it and
  // reintegrates at the sum
  return a.ldlt().solve(b);
}

bool VisualInertialInitializer::linear_alignment(
  std::vector<Eigen::Vector3d> & velocities, Eigen::Vector3d & gravity, double & scale)
{
  // the alignment holds the magnitude the initializer seeded and refines only the direction
  const Eigen::Matrix3d R_bc = T_imu_camera_.block<3, 3>(0, 0);
  const Eigen::Vector3d t_bc = T_imu_camera_.block<3, 1>(0, 3);
  const int all_frame_count = static_cast<int>(frames_.size());
  const int n_state = all_frame_count * 3 + 3 + 1;

  Eigen::MatrixXd a(n_state, n_state);
  Eigen::VectorXd b(n_state);
  a.setZero();
  b.setZero();
  assert(imu_edges_.size() + 1 == frames_.size());
  for (int i = 0; i < static_cast<int>(imu_edges_.size()); i++) {
    const auto & pre_integration = imu_edges_[i].pre_integration;
    const Eigen::Matrix3d R_i = frames_[i].T_c.block<3, 3>(0, 0) * R_bc.transpose();
    const Eigen::Matrix3d R_j = frames_[i + 1].T_c.block<3, 3>(0, 0) * R_bc.transpose();
    const Eigen::Vector3d t_ij =
      frames_[i + 1].T_c.block<3, 1>(0, 3) - frames_[i].T_c.block<3, 1>(0, 3);
    Eigen::MatrixXd tmp_a(6, 10);
    Eigen::VectorXd tmp_b(6);
    tmp_a.setZero();
    tmp_b.setZero();
    const double dt = pre_integration->get_delta_time();
    tmp_a.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
    tmp_a.block<3, 3>(0, 6) = -R_i.transpose() * dt * dt / 2;
    tmp_a.block<3, 1>(0, 9) = R_i.transpose() * t_ij / 100.0;
    tmp_b.block<3, 1>(0, 0) = pre_integration->get_alpha() +
      R_i.transpose() * R_j * t_bc - t_bc;
    tmp_a.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
    tmp_a.block<3, 3>(3, 3) = R_i.transpose() * R_j;
    tmp_a.block<3, 3>(3, 6) = -R_i.transpose() * dt;
    tmp_b.block<3, 1>(3, 0) = pre_integration->get_beta();

    const Eigen::MatrixXd r_a = tmp_a.transpose() * tmp_a;
    const Eigen::VectorXd r_b = tmp_a.transpose() * tmp_b;

    a.block<6, 6>(i * 3, i * 3) += r_a.topLeftCorner<6, 6>();
    b.segment<6>(i * 3) += r_b.head<6>();
    a.bottomRightCorner<4, 4>() += r_a.bottomRightCorner<4, 4>();
    b.tail<4>() += r_b.tail<4>();
    a.block<6, 4>(i * 3, n_state - 4) += r_a.topRightCorner<6, 4>();
    a.block<4, 6>(n_state - 4, i * 3) += r_a.bottomLeftCorner<4, 6>();
  }
  a = a * 1000.0;
  b = b * 1000.0;
  Eigen::VectorXd x = a.ldlt().solve(b);
  scale = x(n_state - 1) / 100.0;
  gravity = x.segment<3>(n_state - 4);
  // a solve that is not a number passes every threshold below, it is refused on its own
  if (!x.allFinite() || std::abs(gravity.norm() - g_norm_) > 1.0 || scale < 0) {
    return false;
  }
  refine_gravity(gravity, x);
  // the equations above scale the sfm structure by 100, the caller expects the metric scale
  scale = (x.tail<1>())(0) / 100.0;
  if (!x.allFinite() || scale < 0.0) {
    return false;
  }
  // one velocity per frame of the window, in the frame that frame is in
  velocities.resize(frames_.size());
  for (int i = 0; i < static_cast<int>(velocities.size()); i++) {
    velocities[i] = x.segment<3>(i * 3);
  }
  return true;
}

void VisualInertialInitializer::refine_gravity(
  Eigen::Vector3d & gravity, Eigen::VectorXd & x)
{
  const Eigen::Matrix3d R_bc = T_imu_camera_.block<3, 3>(0, 0);
  const Eigen::Vector3d t_bc = T_imu_camera_.block<3, 1>(0, 3);
  Eigen::Vector3d g0 = gravity.normalized() * g_norm_;
  const int all_frame_count = static_cast<int>(frames_.size());
  const int n_state = all_frame_count * 3 + 2 + 1;

  Eigen::MatrixXd a(n_state, n_state);
  Eigen::VectorXd b(n_state);
  assert(imu_edges_.size() + 1 == frames_.size());
  for (int k = 0; k < 4; k++) {
    a.setZero();
    b.setZero();
    const Eigen::Matrix<double, 3, 2> lxly = tangent_basis(g0);
    for (int i = 0; i < static_cast<int>(imu_edges_.size()); i++) {
      const auto & pre_integration = imu_edges_[i].pre_integration;
      const Eigen::Matrix3d R_i = frames_[i].T_c.block<3, 3>(0, 0) * R_bc.transpose();
      const Eigen::Matrix3d R_j = frames_[i + 1].T_c.block<3, 3>(0, 0) * R_bc.transpose();
      const Eigen::Vector3d t_ij =
        frames_[i + 1].T_c.block<3, 1>(0, 3) - frames_[i].T_c.block<3, 1>(0, 3);
      Eigen::MatrixXd tmp_a(6, 9);
      Eigen::VectorXd tmp_b(6);
      tmp_a.setZero();
      tmp_b.setZero();
      const double dt = pre_integration->get_delta_time();
      tmp_a.block<3, 3>(0, 0) = -dt * Eigen::Matrix3d::Identity();
      tmp_a.block<3, 2>(0, 6) = -R_i.transpose() * dt * dt / 2 * lxly;
      tmp_a.block<3, 1>(0, 8) = R_i.transpose() * t_ij / 100.0;
      tmp_b.block<3, 1>(0, 0) = pre_integration->get_alpha() +
        R_i.transpose() * R_j * t_bc - t_bc +
        R_i.transpose() * dt * dt / 2 * g0;
      tmp_a.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
      tmp_a.block<3, 3>(3, 3) = R_i.transpose() * R_j;
      tmp_a.block<3, 2>(3, 6) = -R_i.transpose() * dt * lxly;
      tmp_b.block<3, 1>(3, 0) = pre_integration->get_beta() + R_i.transpose() * dt * g0;

      const Eigen::MatrixXd r_a = tmp_a.transpose() * tmp_a;
      const Eigen::VectorXd r_b = tmp_a.transpose() * tmp_b;

      a.block<6, 6>(i * 3, i * 3) += r_a.topLeftCorner<6, 6>();
      b.segment<6>(i * 3) += r_b.head<6>();
      a.bottomRightCorner<3, 3>() += r_a.bottomRightCorner<3, 3>();
      b.tail<3>() += r_b.tail<3>();
      a.block<6, 3>(i * 3, n_state - 3) += r_a.topRightCorner<6, 3>();
      a.block<3, 6>(n_state - 3, i * 3) += r_a.bottomLeftCorner<3, 6>();
    }
    a = a * 1000.0;
    b = b * 1000.0;
    x = a.ldlt().solve(b);
    const Eigen::VectorXd dg = x.segment<2>(n_state - 3);
    g0 = (g0 + lxly * dg).normalized() * g_norm_;
  }
  gravity = g0;
}

Eigen::Matrix<double, 3, 2> VisualInertialInitializer::tangent_basis(
  const Eigen::Vector3d & g0) const
{
  const Eigen::Vector3d a = g0.normalized();
  Eigen::Vector3d tmp(0, 0, 1);
  if (a == tmp) {
    tmp << 1, 0, 0;
  }
  const Eigen::Vector3d b = (tmp - a * (a.transpose() * tmp)).normalized();
  const Eigen::Vector3d c = a.cross(b);
  Eigen::Matrix<double, 3, 2> bc;
  bc.block<3, 1>(0, 0) = b;
  bc.block<3, 1>(0, 1) = c;
  return bc;
}

}  // namespace slt_vio
