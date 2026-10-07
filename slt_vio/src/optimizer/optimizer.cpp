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

#include "slt_vio/optimizer/optimizer.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "slt_vio/optimizer/factor/imu_factor.hpp"
#include "slt_vio/optimizer/factor/projection_factor.hpp"
#include "slt_vio/optimizer/pose_manifold.hpp"
#include "slt_vio/optimizer/prvag_manifold.hpp"

namespace slt_vio
{
Optimizer::Optimizer(const YAML::Node & config)
{
  max_feature_count_ = config["max_feature_count"].as<int>(max_feature_count_);
  max_solver_time_ = config["max_solver_time"].as<double>(max_solver_time_);
  max_num_iterations_ = config["max_num_iterations"].as<int>(max_num_iterations_);
  // one pixel is 1/focal_length on the normalized image (vins' virtual camera)
  const double focal_length = config["focal_length"].as<double>(460.0);
  sqrt_info_ = focal_length / 1.5 * Eigen::Matrix2d::Identity();
  elapsed_time_statistics_.set_enable(config["enable_elapsed_time_statistics"].as<bool>(false));
  elapsed_time_statistics_.set_title("Optimizer");
  clear_state();
}

void Optimizer::clear_state()
{
  frames_.clear();
  landmarks_.clear();
  imu_edges_.clear();
  pending_removal_.clear();
  solved_once_ = false;
  last_marginalization_info_ = nullptr;
  last_marginalization_parameter_blocks_.clear();
}

std::string Optimizer::error_message()
{
  const std::string message = message_;
  message_.clear();
  return message;
}

void Optimizer::set_extrinsic(const Eigen::Matrix4d & T_imu_camera)
{
  T_imu_camera_ = T_imu_camera;
  // the block the factors take: translation, then the so3 log
  Eigen::Map<Eigen::Matrix<double, 6, 1>> param(extrinsic_param_block_.data());
  param.head<3>() = T_imu_camera_.block<3, 1>(0, 3);
  param.tail<3>() = Sophus::SO3d(T_imu_camera_.block<3, 3>(0, 0)).log();
}

void Optimizer::add_frame(
  uint64_t frame_id, const slt_common::ImuNavState & state, bool is_key_frame)
{
  if (frame_id == 0) {
    return;
  }
  // a new frame enters where its id puts it, a held one gets the state it has now
  auto & held = frames_[frame_id];
  if (!held) {
    held = std::make_shared<Frame>();
  }
  Frame & frame = *held;
  frame.time = state.time;
  frame.is_key_frame = is_key_frame;
  frame.gravity = state.gravity;
  Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_P) = state.position;
  Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_R) = Sophus::SO3d(state.orientation).log();
  Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_V) = state.linear_velocity;
  Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_BA) = state.accel_bias;
  Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_BG) = state.gyro_bias;
}

void Optimizer::add_imu_edge(
  uint64_t start_frame_id, uint64_t end_frame_id,
  const slt_imu_odometry::ImuPreIntegrationPtr & pre_integration)
{
  // an interval with no preintegration, or one too long for the window, never enters
  if (!pre_integration || pre_integration->get_delta_time() > kMaxPreIntegrationTime) {
    return;
  }
  imu_edges_.push_back(ImuEdge{start_frame_id, end_frame_id, pre_integration});
}

void Optimizer::add_feature(const FeatureFrame & feature_frame)
{
  // a frame the window does not hold any more is refused here
  if (!frames_.count(feature_frame.frame_id)) {
    return;
  }
  for (const auto & id_pts : feature_frame.features) {
    auto it = landmarks_.find(id_pts.first);
    if (it == landmarks_.end()) {
      // a new landmark is refused once the graph is full, an old one still takes its observation
      if (static_cast<int>(landmarks_.size()) >= max_feature_count_) {
        message_ = "the graph is full, a feature beyond " + std::to_string(max_feature_count_) +
          " observations is dropped";
        continue;
      }
      it = landmarks_.emplace(id_pts.first, std::make_shared<Landmark>()).first;
    }
    it->second->observations[feature_frame.frame_id] = id_pts.second.xy;
  }
}

void Optimizer::collect_removed_features()
{
  // a block the prior still points at stays until a prior forgets it, without observations
  auto it = pending_removal_.begin();
  while (it != pending_removal_.end()) {
    const auto feature_it = landmarks_.find(*it);
    if (feature_it == landmarks_.end()) {
      it = pending_removal_.erase(it);
      continue;
    }
    const bool kept =
      std::count(
      last_marginalization_parameter_blocks_.begin(),
      last_marginalization_parameter_blocks_.end(),
      &feature_it->second->inv_depth) > 0;
    if (kept) {
      it++;
      continue;
    }
    // a live feature cannot be reaped: only observation-less ones are listed, and ids do not return
    assert(feature_it->second->observations.empty());
    landmarks_.erase(feature_it);
    it = pending_removal_.erase(it);
  }
}

double Optimizer::triangulate_landmark(const Landmark & landmark) const
{
  // the extrinsic rotation and translation, folded into every observation's camera
  const Eigen::Matrix3d R_bc = T_imu_camera_.block<3, 3>(0, 0);
  const Eigen::Vector3d t_bc = T_imu_camera_.block<3, 1>(0, 3);
  // the observing frames oldest first; the window holds every frame a landmark observed
  std::vector<std::pair<const Frame *, Eigen::Vector3d>> seen;
  seen.reserve(landmark.observations.size());
  for (const auto & frame : frames_) {
    const auto observation = landmark.observations.find(frame.first);
    if (observation != landmark.observations.end()) {
      seen.emplace_back(frame.second.get(), observation->second.homogeneous());
    }
  }
  // the oldest observing frame's camera, the one every observation is taken into
  const Frame & anchor = *seen.front().first;
  const Eigen::Map<const Eigen::Vector3d> anchor_p(anchor.data + INDEX_P);
  const Eigen::Map<const Eigen::Vector3d> anchor_r(anchor.data + INDEX_R);
  const Eigen::Matrix3d anchor_R = Sophus::SO3d::exp(anchor_r).matrix();
  const Eigen::Vector3d t0 = anchor_p + anchor_R * t_bc;
  const Eigen::Matrix3d R0 = anchor_R * R_bc;
  Eigen::MatrixXd svd_a(2 * seen.size(), 4);
  int svd_idx = 0;
  for (const auto & observation : seen) {
    const Frame & frame = *observation.first;
    const Eigen::Map<const Eigen::Vector3d> p(frame.data + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> r(frame.data + INDEX_R);
    const Eigen::Matrix3d R = Sophus::SO3d::exp(r).matrix();
    const Eigen::Vector3d t1 = p + R * t_bc;
    const Eigen::Matrix3d R1 = R * R_bc;
    const Eigen::Vector3d t = R0.transpose() * (t1 - t0);
    const Eigen::Matrix3d rel_R = R0.transpose() * R1;
    Eigen::Matrix<double, 3, 4> proj;
    proj.leftCols<3>() = rel_R.transpose();
    proj.rightCols<1>() = -rel_R.transpose() * t;
    const Eigen::Vector3d f = observation.second.normalized();
    svd_a.row(svd_idx++) = f[0] * proj.row(2) - f[2] * proj.row(0);
    svd_a.row(svd_idx++) = f[1] * proj.row(2) - f[2] * proj.row(1);
  }
  const Eigen::Vector4d svd_v =
    Eigen::JacobiSVD<Eigen::MatrixXd>(svd_a, Eigen::ComputeThinV).matrixV().rightCols<1>();
  return svd_v[2] / svd_v[3];
}

void Optimizer::triangulate()
{
  for (auto & entry : landmarks_) {
    Landmark & landmark = *entry.second;
    // three views before a depth is solved for; a feature with one is the solve's to refine
    if (landmark.observations.size() < 3 || landmark.state != Landmark::State::kNoDepth) {
      continue;
    }
    // the anchor camera: in the window, and not among the newest 3, where there is no baseline yet
    const auto anchor = frames_.find(landmark.observations.begin()->first);
    if (anchor == frames_.end() ||
      static_cast<size_t>(std::distance(anchor, frames_.end())) <= 3)
    {
      continue;
    }
    const double depth = triangulate_landmark(landmark);
    if (!std::isfinite(depth) || depth < 0.1) {
      // a bad track: the observations go now, the block when a prior stops naming it
      landmark.state = Landmark::State::kFailed;
      landmark.observations.clear();
      pending_removal_.insert(entry.first);
      continue;
    }
    landmark.inv_depth = 1.0 / depth;
    landmark.state = Landmark::State::kHasDepth;
  }
  collect_removed_features();
}

void Optimizer::optimize()
{
  if (frames_.empty()) {
    return;
  }
  elapsed_time_statistics_.tic("optimize");
  // features with no depth are triangulated now, ones with a depth refined by the solve
  triangulate();
  ceres::Problem problem;
  // the problem owns the loss and the manifold, deleting a shared one only once
  auto * loss_function = new ceres::CauchyLoss(1.0);
  auto * state_manifold = new PrvagManifold();
  for (auto & entry : frames_) {
    Frame & frame = *entry.second;
    problem.AddParameterBlock(frame.data, 15, state_manifold);
  }
  // the extrinsic comes from the config, it is never estimated
  problem.AddParameterBlock(extrinsic_param_block_.data(), 6, new PoseManifold());
  problem.SetParameterBlockConstant(extrinsic_param_block_.data());

  // the world frame the solve starts from, the result is anchored back onto it
  const Frame origin = *frames_.begin()->second;

  if (last_marginalization_info_) {
    auto * marginalization_factor = new MarginalizationFactor(last_marginalization_info_.get());
    problem.AddResidualBlock(
      marginalization_factor, nullptr, last_marginalization_parameter_blocks_);
  }
  // the imu residuals: one per interval of the window, between the two frames it names
  for (const ImuEdge & edge : imu_edges_) {
    const auto from = frames_.find(edge.from);
    const auto to = frames_.find(edge.to);
    // the interval enters when the window holds both frames it names, sitting next to each other
    if (from == frames_.end() || to == frames_.end() || std::next(from) != to) {
      continue;
    }
    Frame & frame0 = *from->second;
    Frame & frame1 = *to->second;
    auto * imu_factor = new ImuFactor(edge.pre_integration, frame0.gravity);
    problem.AddResidualBlock(
      imu_factor, nullptr, frame0.data, frame1.data);
  }
  // and the visual ones: a feature, anchored at its oldest observation, against the frames after
  for (auto & entry : landmarks_) {
    Landmark & landmark = *entry.second;
    // two frames see it: one anchors the depth, the other carries the residual
    if (landmark.observations.size() < 2 || landmark.state != Landmark::State::kHasDepth) {
      continue;
    }
    const auto anchor = landmark.observations.begin();
    const auto anchor_frame = frames_.find(anchor->first);
    if (anchor_frame == frames_.end()) {
      continue;
    }
    problem.AddParameterBlock(&landmark.inv_depth, 1);
    const Eigen::Vector3d pts_i = anchor->second.homogeneous();
    for (auto it = std::next(anchor); it != landmark.observations.end(); ++it) {
      const auto frame = frames_.find(it->first);
      if (frame == frames_.end()) {
        continue;
      }
      auto * factor = new ProjectionFactor(pts_i, it->second.homogeneous(), sqrt_info_);
      problem.AddResidualBlock(
        factor, loss_function, anchor_frame->second->data, frame->second->data,
        extrinsic_param_block_.data(), &landmark.inv_depth);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = max_num_iterations_;
  options.max_solver_time_in_seconds = max_solver_time_;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  apply_solution(origin);
  // a depth the solve put behind the camera is invalid: none again, for the next triangulation
  for (auto & entry : landmarks_) {
    // a depth that is not a number any more is as invalid as a negative one
    if (entry.second->state == Landmark::State::kHasDepth &&
      (!std::isfinite(entry.second->inv_depth) || entry.second->inv_depth <= 0.0))
    {
      entry.second->inv_depth = 0.0;
      entry.second->state = Landmark::State::kNoDepth;
    }
  }
  solved_once_ = true;
  elapsed_time_statistics_.toc("optimize");
  elapsed_time_statistics_.print_all_info("optimize", 100);
}

void Optimizer::apply_solution(const Frame & origin)
{
  // the residuals do not see yaw or position: the window is turned back onto the first frame's
  // anchors are read as values, the loop mutates the states as it goes
  const Eigen::Vector3d origin_p = Eigen::Map<const Eigen::Vector3d>(origin.data + INDEX_P);
  const Eigen::Vector3d origin_r = Eigen::Map<const Eigen::Vector3d>(origin.data + INDEX_R);
  const double * solved_state = frames_.begin()->second->data;
  const Eigen::Vector3d solved_p = Eigen::Map<const Eigen::Vector3d>(solved_state + INDEX_P);
  const Eigen::Vector3d solved_r = Eigen::Map<const Eigen::Vector3d>(solved_state + INDEX_R);
  const Eigen::Matrix3d origin_R = Sophus::SO3d::exp(origin_r).matrix();
  const Eigen::Matrix3d solved_R = Sophus::SO3d::exp(solved_r).matrix();
  const Eigen::Vector3d origin_r0 = slt_common::R_to_ypr(origin_R);
  const Eigen::Vector3d solved_r0 = slt_common::R_to_ypr(solved_R);
  const double y_diff = origin_r0.x() - solved_r0.x();

  // only the yaw is free, the roll and the pitch are held by the gravity of the imu residual
  Eigen::Matrix3d R_diff = slt_common::ypr_to_R(Eigen::Vector3d(y_diff, 0, 0));
  if (std::abs(std::abs(origin_r0.y()) - 90) < 1.0 ||
    std::abs(std::abs(solved_r0.y()) - 90) < 1.0)
  {
    // the yaw is not observable at the euler singular point, take the rotation from the solve
    R_diff = origin_R * solved_R.transpose();
  }

  for (auto & entry : frames_) {
    Frame & frame = *entry.second;
    const Eigen::Map<const Eigen::Vector3d> p(frame.data + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> r(frame.data + INDEX_R);
    const Eigen::Map<const Eigen::Vector3d> v(frame.data + INDEX_V);
    const Eigen::Matrix3d R = R_diff * Sophus::SO3d::exp(r).matrix();
    const Eigen::Vector3d position = R_diff * (p - solved_p) + origin_p;
    // velocity is world frame and turns with the window; biases and depths are body frame and stay
    const Eigen::Vector3d velocity = R_diff * v;
    Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_P) = position;
    Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_R) = Sophus::SO3d(R).log();
    Eigen::Map<Eigen::Vector3d>(frame.data + INDEX_V) = velocity;
  }
}

void Optimizer::drop_observations(uint64_t frame_id)
{
  for (auto & entry : landmarks_) {
    Landmark & landmark = *entry.second;
    // the observation goes with the frame; the feature stays while one is left
    if (landmark.observations.erase(frame_id) > 0 && landmark.observations.empty()) {
      pending_removal_.insert(entry.first);
    }
  }
}

void Optimizer::drop_and_reanchor_observations(uint64_t frame_id)
{
  // the camera the depths were measured in
  const auto leaving = frames_.find(frame_id);
  if (leaving == frames_.end()) {
    return;
  }
  const Eigen::Map<const Eigen::Vector3d> back_p(leaving->second->data + INDEX_P);
  const Eigen::Map<const Eigen::Vector3d> back_r(leaving->second->data + INDEX_R);
  const Eigen::Matrix3d back_R = Sophus::SO3d::exp(back_r).matrix();
  const Eigen::Matrix3d marg_R = back_R * T_imu_camera_.block<3, 3>(0, 0);
  const Eigen::Vector3d marg_p = back_p + back_R * T_imu_camera_.block<3, 1>(0, 3);
  for (auto & entry : landmarks_) {
    Landmark & landmark = *entry.second;
    // the leaving frame is the oldest: its observation sits at the front and carries the depth
    if (landmark.observations.empty() || landmark.observations.begin()->first != frame_id) {
      continue;
    }
    const Eigen::Vector3d pts_i = landmark.observations.begin()->second.homogeneous();
    landmark.observations.erase(landmark.observations.begin());
    if (landmark.observations.empty()) {
      pending_removal_.insert(entry.first);
      continue;
    }
    if (landmark.observations.size() == 1) {
      // a single view cannot triangulate: no depth until another one arrives
      landmark.inv_depth = 0.0;
      landmark.state = Landmark::State::kNoDepth;
      continue;
    }
    // the depth moves into the camera of the observation it anchors at now
    const auto new_anchor = frames_.find(landmark.observations.begin()->first);
    if (new_anchor == frames_.end() || landmark.state != Landmark::State::kHasDepth) {
      // a depth that cannot be moved is none again, for the triangulation
      landmark.inv_depth = 0.0;
      landmark.state = Landmark::State::kNoDepth;
      continue;
    }
    const double * new_state = new_anchor->second->data;
    const Eigen::Map<const Eigen::Vector3d> new_p(new_state + INDEX_P);
    const Eigen::Map<const Eigen::Vector3d> new_r(new_state + INDEX_R);
    const Eigen::Matrix3d new_R = Sophus::SO3d::exp(new_r).matrix();
    const Eigen::Matrix3d new_cam_R = new_R * T_imu_camera_.block<3, 3>(0, 0);
    const Eigen::Vector3d new_cam_p = new_p + new_R * T_imu_camera_.block<3, 1>(0, 3);
    const Eigen::Vector3d w_pts_i = marg_R * (pts_i / landmark.inv_depth) + marg_p;
    const Eigen::Vector3d pts_j = new_cam_R.transpose() * (w_pts_i - new_cam_p);
    // a depth the move computed behind the camera is invalid: none again, for the triangulation
    if (pts_j(2) > 0.0) {
      landmark.inv_depth = 1.0 / pts_j(2);
    } else {
      landmark.inv_depth = 0.0;
      landmark.state = Landmark::State::kNoDepth;
    }
  }
}

void Optimizer::marginalize()
{
  elapsed_time_statistics_.tic("marginalize");
  if (frames_.size() >= 2) {
    // a key frame at stake lets the oldest go, one that is not is the frame that leaves
    if (std::prev(frames_.end(), 2)->second->is_key_frame) {
      // before the first solve there is no prior and no depth: the frame only leaves
      if (solved_once_) {
        marginalize_old();
      } else {
        drop_old();
      }
    } else {
      marginalize_new();
    }
  }
  elapsed_time_statistics_.toc("marginalize");
}

void Optimizer::marginalize_old()
{
  if (frames_.empty()) {
    return;
  }
  elapsed_time_statistics_.tic("marginalize_old");
  const auto front = frames_.begin();
  const uint64_t leaving_frame_id = front->first;

  // the visual residuals are linearized here, with this loss, and they are gone once it returns
  ceres::CauchyLoss loss_function(1.0);
  auto marginalization_info = std::make_shared<MarginalizationInfo>();

  if (last_marginalization_info_) {
    std::vector<int> drop_set;
    for (size_t i = 0; i < last_marginalization_parameter_blocks_.size(); i++) {
      if (last_marginalization_parameter_blocks_[i] == front->second->data) {
        drop_set.push_back(static_cast<int>(i));
      }
    }
    auto * marginalization_factor = new MarginalizationFactor(last_marginalization_info_.get());
    marginalization_info->add_residual_block_info(
      std::make_unique<ResidualBlockInfo>(
        marginalization_factor, nullptr, last_marginalization_parameter_blocks_, drop_set));
  }

  // the interval the frame leaves with is the head of the log: it goes with it
  slt_imu_odometry::ImuPreIntegrationPtr opening;
  if (frames_.size() > 1 && !imu_edges_.empty() &&
    imu_edges_.front().from == leaving_frame_id && imu_edges_.front().to == std::next(front)->first)
  {
    opening = imu_edges_.front().pre_integration;
    imu_edges_.pop_front();
  }
  if (opening) {
    auto * imu_factor = new ImuFactor(opening, front->second->gravity);
    marginalization_info->add_residual_block_info(
      std::make_unique<ResidualBlockInfo>(
        imu_factor, nullptr,
        std::vector<double *>{
        front->second->data, std::next(front)->second->data},
        std::vector<int>{0}));
  }

  // the next prior's visual residuals: those anchored at the leaving frame, so what it saw stays
  for (auto & entry : landmarks_) {
    Landmark & landmark = *entry.second;
    const auto anchor = landmark.observations.find(leaving_frame_id);
    if (anchor == landmark.observations.end()) {
      continue;
    }
    // a point without a usable depth has nothing to linearize
    if (landmark.state != Landmark::State::kHasDepth) {
      continue;
    }
    const Eigen::Vector3d pts_i = anchor->second.homogeneous();
    for (const auto & observation : landmark.observations) {
      const auto frame = frames_.find(observation.first);
      if (frame == frames_.end() || observation.first == leaving_frame_id) {
        continue;
      }
      auto * factor = new ProjectionFactor(pts_i, observation.second.homogeneous(), sqrt_info_);
      marginalization_info->add_residual_block_info(
        std::make_unique<ResidualBlockInfo>(
          factor, &loss_function,
          std::vector<double *>{
          front->second->data, frame->second->data, extrinsic_param_block_.data(),
          &landmark.inv_depth},
          std::vector<int>{0, 3}));
    }
  }

  marginalization_info->marginalize();

  // the window front leaves and nothing else moves: the prior's blocks stay where they are
  last_marginalization_parameter_blocks_ = marginalization_info->get_parameter_blocks();
  last_marginalization_info_ = marginalization_info;

  // the depths move out while the frame is still here to name their camera
  drop_and_reanchor_observations(leaving_frame_id);

  // the window slides: the front is gone with the interval it opened, the ones behind do not move
  frames_.erase(front);
  collect_removed_features();
  elapsed_time_statistics_.toc("marginalize_old");
}

void Optimizer::drop_old()
{
  if (frames_.empty()) {
    return;
  }
  const uint64_t leaving_frame_id = frames_.begin()->first;
  // the window slides: the front is gone with the interval that ended at it, the rest do not move
  frames_.erase(frames_.begin());
  // the intervals of the frames that left go with them
  while (!imu_edges_.empty() &&
    (!frames_.count(imu_edges_.front().from) || !frames_.count(imu_edges_.front().to)))
  {
    imu_edges_.pop_front();
  }
  // the frame took its interval with it, and nothing points at what it saw any more
  for (auto it = landmarks_.begin(); it != landmarks_.end(); ) {
    Landmark & landmark = *it->second;
    if (landmark.observations.erase(leaving_frame_id) > 0 && landmark.observations.empty()) {
      it = landmarks_.erase(it);
    } else {
      ++it;
    }
  }
}

void Optimizer::marginalize_new()
{
  if (frames_.size() < 2) {
    return;
  }
  elapsed_time_statistics_.tic("marginalize_new");
  const auto leaving = std::prev(frames_.end(), 2);
  const uint64_t leaving_frame_id = leaving->first;
  const uint64_t newest_frame_id = frames_.rbegin()->first;

  // the two intervals the log ends with become one: edge1's samples go onto edge0
  if (imu_edges_.size() >= 2) {
    const ImuEdge edge0 = imu_edges_[imu_edges_.size() - 2];
    const ImuEdge edge1 = imu_edges_.back();
    if (edge0.to == leaving_frame_id && edge1.from == leaving_frame_id &&
      edge1.to == newest_frame_id)
    {
      // edge1's first sample is the one edge0 already ends on, and integrate() drops it
      for (const auto & sample : edge1.pre_integration->get_imu_data_buffer()) {
        edge0.pre_integration->integrate(sample);
      }
      // edge1 goes, the merged edge enters the way every interval does
      imu_edges_.pop_back();
      imu_edges_.pop_back();
      add_imu_edge(edge0.from, newest_frame_id, edge0.pre_integration);
    }
  }

  // the second newest is dropped from the prior, the newest takes its place
  // no prior: nothing to drop and no depth to keep
  if (last_marginalization_info_ &&
    std::count(
      last_marginalization_parameter_blocks_.begin(), last_marginalization_parameter_blocks_.end(),
      leaving->second->data))
  {
    auto marginalization_info = std::make_shared<MarginalizationInfo>();

    std::vector<int> drop_set;
    for (size_t i = 0; i < last_marginalization_parameter_blocks_.size(); i++) {
      if (last_marginalization_parameter_blocks_[i] == leaving->second->data) {
        drop_set.push_back(static_cast<int>(i));
      }
    }
    auto * marginalization_factor = new MarginalizationFactor(last_marginalization_info_.get());
    marginalization_info->add_residual_block_info(
      std::make_unique<ResidualBlockInfo>(
        marginalization_factor, nullptr, last_marginalization_parameter_blocks_, drop_set));

    marginalization_info->marginalize();

    last_marginalization_parameter_blocks_ = marginalization_info->get_parameter_blocks();
    last_marginalization_info_ = marginalization_info;
  }

  // the leaving frame goes out of the middle, the newest stays at the back: no block moves, so the
  // prior's blocks are still its own
  frames_.erase(leaving);

  drop_observations(leaving_frame_id);
  collect_removed_features();
  elapsed_time_statistics_.toc("marginalize_new");
}

slt_common::ImuNavState Optimizer::get_imu_nav_state() const
{
  if (frames_.empty()) {
    return {};
  }
  return create_nav_state(*frames_.rbegin()->second);
}

slt_common::ImuNavState Optimizer::create_nav_state(const Frame & frame) const
{
  slt_common::ImuNavState state;
  state.time = frame.time;
  const Eigen::Map<const Eigen::Vector3d> p(frame.data + INDEX_P);
  const Eigen::Map<const Eigen::Vector3d> r(frame.data + INDEX_R);
  const Eigen::Map<const Eigen::Vector3d> v(frame.data + INDEX_V);
  const Eigen::Map<const Eigen::Vector3d> ba(frame.data + INDEX_BA);
  const Eigen::Map<const Eigen::Vector3d> bg(frame.data + INDEX_BG);
  state.position = p;
  state.orientation = Sophus::SO3d::exp(r).matrix();
  state.linear_velocity = v;
  state.accel_bias = ba;
  state.gyro_bias = bg;
  state.gravity = frame.gravity;
  return state;
}

bool Optimizer::has_landmark(int feature_id) const
{
  const auto it = landmarks_.find(feature_id);
  return it != landmarks_.end() && !it->second->observations.empty();
}
}  // namespace slt_vio
