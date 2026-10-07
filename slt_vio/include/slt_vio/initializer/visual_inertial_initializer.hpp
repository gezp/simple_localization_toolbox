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
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "slt_common/math_utils.hpp"
#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/tic_toc.hpp"
#include "slt_imu_odometry/imu_pre_integration.hpp"
#include "slt_vio/feature.hpp"
#include "yaml-cpp/yaml.h"

namespace slt_vio
{
// one frame of the initializer's window: tracker features, the sfm camera pose, the aligned state
struct ImageFrame
{
  ImageFrame() = default;
  ImageFrame(const std::map<int, FeatureObservation> & features, double time)
  : features(features), time(time) {}

  std::map<int, FeatureObservation> features;
  // the id the graph keys this frame by, the time stamp of its image in nanoseconds
  uint64_t frame_id{0};
  double time{0.0};
  // the camera pose the sfm settled on
  Eigen::Matrix4d T_c{Eigen::Matrix4d::Identity()};
  // the state the alignment settled on this frame
  slt_common::ImuNavState state;
  // the window drops the oldest frame, or this one when it is not a key frame
  bool is_key_frame{true};
};

// the imu edge between two window frames, opened by the sample the first one ended on: one fewer
// than the frames, oldest opening none, in join order, the accumulator's samples merging onto it
struct ImuEdge
{
  // the two frames it joins, by the time stamp of each image, the ids the graph keys frames by
  uint64_t from{0};
  uint64_t to{0};
  slt_imu_odometry::ImuPreIntegrationPtr pre_integration;
};

// the bootstrap of the sliding window estimator: it owns the initialization's frame window as one
// deque, rolls it until the sfm and alignment settle scale, gravity and biases, then hands it over.
// the caller owns the imu stream; this class never sees a sample nor judges a frame
class VisualInertialInitializer
{
public:
  // reads its keys from its own section, the ones shared with the estimator repeated there
  explicit VisualInertialInitializer(const YAML::Node & config);

  // the camera to imu extrinsic; the poses the alignment settles on are the imu ones. set before
  // the first frame
  void set_extrinsic(const Eigen::Matrix4d & T_imu_camera);

  // one tracker output frame, with the preintegration ending at its time stamp and the key frame
  // verdict: it enters on the edge the frame before opened; the frame opening a window closes none
  void add_frame(
    const FeatureFrame & feature_frame,
    const slt_imu_odometry::ImuPreIntegrationPtr & pre_integration, bool is_key_frame);
  // attempts the window on the frames entered since the last call, the settling frame not taken (it
  // belongs to whoever takes the settled window over); once true the getters hold until reset()
  bool try_init();

  // the window in arrival order, oldest first: every frame carries the state the alignment settled
  // on it and the features the graph is born on
  const std::deque<ImageFrame> & frames() const {return frames_;}
  // the imu edges in the order of the frames they join: one fewer than frames, the oldest opening
  // none, each linearized at the bias the alignment solved
  const std::deque<ImuEdge> & imu_edges() const {return imu_edges_;}

  // drains the message raised since the last call (the caller owns the text, this clears it)
  std::string error_message();

  // back to an empty window: the caller reboots, or takes over the settled one
  void reset();

private:
  // the gravity each edge measures has to vary, else the alignment means nothing; false only warned
  bool check_imu_observibility();
  // the global sfm over the window frames: false when there is no relative pose or the sfm fails
  bool structure_from_motion();
  // how far the rotation the edges predict is off the sfm's: the edges are linearized at the bias
  // they carry, so this is the increment they are short by, not the bias itself
  Eigen::Vector3d solve_gyroscope_bias();
  // gravity, velocity and scale of the window solved from its edges; gravity is refined on its own
  // tangent plane, one velocity per frame in that frame's own frame, in window order
  bool linear_alignment(
    std::vector<Eigen::Vector3d> & velocities, Eigen::Vector3d & gravity, double & scale);
  void refine_gravity(Eigen::Vector3d & gravity, Eigen::VectorXd & x);
  // the two vectors spanning the plane the gravity is refined in
  Eigen::Matrix<double, 3, 2> tangent_basis(const Eigen::Vector3d & g0) const;

private:
  // the initialization is retried every time the window moves, but not more often than this
  static constexpr double kInitializationGap = 0.1;
  // below this parallax (pixels of the virtual camera) two frames cannot start the sfm: the sfm
  // reads the normalized image, so the threshold goes to it divided by the focal
  static constexpr double kInitParallaxPx = 30.0;
  int window_size_{10};
  // the gravity magnitude, its direction is refined by the alignment
  double g_norm_{9.81007};
  // the focal length of the virtual camera, it scales the parallax of the initialization
  double focal_length_{460.0};
  // the camera to imu extrinsic, held whole: a frame's pose and the alignment both read it
  Eigen::Matrix4d T_imu_camera_{Eigen::Matrix4d::Identity()};

  // the window in arrival order; a frame that leaves the window leaves the deque
  std::deque<ImageFrame> frames_;
  // the imu edges in the order of the frames they join: one fewer, the oldest opening none
  std::deque<ImuEdge> imu_edges_;

  // the bias every edge is reintegrated at, accumulated from the solve; the next measures from it
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_;
  // the newest frame the sfm of an attempt ran on, the attempts are held to a gap
  double last_attempt_time_{0.0};
  bool initialized_{false};

  std::string message_;
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};
}  // namespace slt_vio
