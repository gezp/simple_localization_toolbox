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
#include <map>
#include <vector>

namespace slt_vio
{
// how the structure from motion made up its mind, the reason reconstruct() turned it down
enum class SfmStatus
{
  kOk,
  kNoRelativePose,  // no two frames of the window carry enough correspondence and parallax
  kPnpFailed,       // a frame could not be placed from the points already triangulated
  kBaFailed,        // the bundle adjustment did not converge
};

// structure from motion over the whole window: it bootstraps the camera poses and feature positions
// the visual inertial alignment needs, reads frames as normalized image points, settles a pose on
// every one of them. nothing of the window, the imu or the estimator reaches it
class GlobalSfm
{
public:
  // the parallax a pair of frames has to carry to start the sfm, in the normalized image: it is the
  // pixel threshold of the initialization over the focal length of the virtual camera
  explicit GlobalSfm(double min_init_parallax);

  // true once settled, frame_poses carries a pose per frame in the order handed over; each input
  // frame hands over every feature keyed by id. false: read the rejecting step off status()
  bool reconstruct(
    const std::vector<std::map<int, Eigen::Vector2d>> & frames,
    std::vector<Eigen::Matrix4d> & frame_poses);

  // the reason the last reconstruct() made up its mind: kOk after a true, else which step refused
  SfmStatus status() const {return status_;}

private:
  // the seed frame: the oldest frame with enough correspondence and parallax to the newest one; the
  // relative pose between the two, handed back alongside, starts the sfm
  bool find_seed_frame(
    const std::vector<std::map<int, Eigen::Vector2d>> & frames, int & seed_index,
    Eigen::Matrix4d & relative_pose) const;

  SfmStatus status_{SfmStatus::kOk};
  double min_init_parallax_;
};
}  // namespace slt_vio
