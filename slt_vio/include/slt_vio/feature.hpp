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
#include <map>

namespace slt_vio
{
// one observation of a feature, as the tracker hands it over: the pixel uv, the undistorted
// normalized coordinate xy, and v_xy its rate, what the time offset estimation needs
struct FeatureObservation
{
  Eigen::Vector2d uv;
  Eigen::Vector2d xy;
  Eigen::Vector2d v_xy;
};

// one output frame of the tracker: the graph's key for it, its image time stamp and what it saw;
// the initializer holds it back, its samples taken only when try_init() processes it
struct FeatureFrame
{
  uint64_t frame_id{0};
  double time{0.0};
  std::map<int, FeatureObservation> features;
};
}  // namespace slt_vio
