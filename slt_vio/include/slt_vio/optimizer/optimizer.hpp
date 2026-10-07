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

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "slt_common/math_utils.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/tic_toc.hpp"
#include "slt_imu_odometry/imu_pre_integration.hpp"
#include "slt_vio/feature.hpp"
#include "slt_vio/optimizer/factor/marginalization_factor.hpp"
#include "yaml-cpp/yaml.h"

namespace slt_vio
{
// the ceres half of the sliding window: one 15-dim state block per frame, one inverse depth block
// per feature, plus the prior the last marginalization left. the prior holds raw pointers into the
// blocks, so a block never moves: frames key by image time stamp in ns, features by tracker id
class Optimizer
{
  // the frame state block slots: p, the so3 log of r, v, accel bias, gyro bias
  static constexpr int INDEX_P = 0;
  static constexpr int INDEX_R = 3;
  static constexpr int INDEX_V = 6;
  static constexpr int INDEX_BA = 9;
  static constexpr int INDEX_BG = 12;

public:
  // one window frame: its 15-dim data block is the prvag layout
  struct Frame
  {
    double time{0.0};
    bool is_key_frame{true};
    double data[15]{};
    Eigen::Vector3d gravity{Eigen::Vector3d{0, 0, -9.81}};
  };

  // one window landmark: the observations and the inverse depth the solve reads
  struct Landmark
  {
    // the situation of the point: no depth yet (or one thrown away for the triangulation to redo),
    // a usable depth, or a triangulation that landed behind the camera and takes the point out
    enum class State
    {
      kNoDepth,
      kHasDepth,
      kFailed,
    };
    std::map<uint64_t, Eigen::Vector2d> observations;
    State state{State::kNoDepth};
    double inv_depth{0.0};
  };

  // one window interval: the two frames it joins and the preintegration between them
  struct ImuEdge
  {
    uint64_t from{0};
    uint64_t to{0};
    slt_imu_odometry::ImuPreIntegrationPtr pre_integration;
  };

  // reads its keys from the `optimizer` config subsection, the focal length of the virtual camera
  // with them
  explicit Optimizer(const YAML::Node & config);

  // the camera to imu extrinsic, a problem block that is never estimated
  void set_extrinsic(const Eigen::Matrix4d & T_imu_camera);
  // a new frame enters at the back, a held one gets the state it has now
  void add_frame(uint64_t frame_id, const slt_common::ImuNavState & state, bool is_key_frame);
  // the preintegration between two frames of the window, both registered
  void add_imu_edge(
    uint64_t start_frame_id, uint64_t end_frame_id,
    const slt_imu_odometry::ImuPreIntegrationPtr & pre_integration);
  // a frame's observations, added at once; a feature is created on its first observation with no
  // depth yet, the triangulation gives it one
  void add_feature(const FeatureFrame & feature_frame);

  // solves the window: prior, imu edges, visual residuals, then the anchor rotation. the window is
  // turned back onto its first frame's yaw and position, or the world frame would drift per solve
  void optimize();
  // the frame at stake is the second newest, its verdict picks which one leaves
  void marginalize();

  // back to an empty window: no block filled, no prior left
  void clear_state();
  // drains the errors raised since the last call: the caller owns the text, this clears it
  std::string error_message();

  // the window's frames, oldest to newest; caller sizes the window, the capacity is its own
  size_t frame_count() const {return frames_.size();}
  // the newest frame as a nav state, a default one when the window holds none
  slt_common::ImuNavState get_imu_nav_state() const;
  // whether the graph holds the landmark the feature id names, with observations still on it
  bool has_landmark(int feature_id) const;

private:
  // the frame as a nav state, with the gravity of the window it belongs to
  slt_common::ImuNavState create_nav_state(const Frame & frame) const;

  // the oldest frame leaves, its features' depths re-anchored on the frame that takes its place
  void marginalize_old();
  // the oldest frame leaves with the intervals that opened at it and its observations; the frames
  // behind it do not move, and the depths it anchored are the caller's to settle
  void drop_old();
  // the second newest frame leaves, its observations go with it and the newest takes its place
  void marginalize_new();
  // triangulates the features that have no depth yet, from the poses the window is on now; a point
  // it cannot place is a bad track and goes here, its feature with it
  void triangulate();
  // one feature's depth from the current poses: one row per observation into the oldest slot's
  // camera, svd null space is the point; the depth can land behind the camera, which fails
  double triangulate_landmark(const Landmark & landmark) const;
  // turns the solved window back onto the world frame its first frame keeps
  void apply_solution(const Frame & origin);
  // the observations of a frame that left the window
  void drop_observations(uint64_t frame_id);
  // the leaving frame's observations go, its depths move into the new anchor's camera
  void drop_and_reanchor_observations(uint64_t frame_id);
  // the blocks of the features nothing points at any more
  void collect_removed_features();

private:
  // a preintegration longer than this is not trusted, the window did not cover the gap
  static constexpr double kMaxPreIntegrationTime = 10.0;
  int max_feature_count_{1000};
  double max_solver_time_{0.04};
  int max_num_iterations_{8};

  // the visual residual weight, vins uses the focal length of the virtual camera
  Eigen::Matrix2d sqrt_info_;
  // the camera to imu extrinsic, whole
  Eigen::Matrix4d T_imu_camera_{Eigen::Matrix4d::Identity()};
  // the same extrinsic as the factors take it: translation, then the so3 log
  std::array<double, 6> extrinsic_param_block_{};

  // the window: frames keyed by id, each behind its own pointer, so a block never moves
  std::map<uint64_t, std::shared_ptr<Frame>> frames_;
  // feature id -> observations and depth block, each landmark behind its own pointer so the block
  // never moves
  std::map<int, std::shared_ptr<Landmark>> landmarks_;
  // the window intervals, named by the two frames they join; the solve looks them up by id, and a
  // leaving frame takes its intervals with it
  std::deque<ImuEdge> imu_edges_;
  // features without observations, waiting for their block to be collected
  std::set<int> pending_removal_;
  // set by the first solve: before it the window holds no depth any marginalization could keep
  bool solved_once_{false};

  std::shared_ptr<MarginalizationInfo> last_marginalization_info_;
  std::vector<double *> last_marginalization_parameter_blocks_;

  std::string message_;
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};
}  // namespace slt_vio
