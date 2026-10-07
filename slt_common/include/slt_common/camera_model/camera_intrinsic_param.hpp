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

#pragma once

#include <string>
#include <vector>

namespace slt_common
{
struct CameraIntrinsicParam
{
  std::string frame_id;
  int height = 0;
  int width = 0;
  // only "plumb_bob" is supported, see CameraModel
  std::string distortion_model;
  // fx, fy, cx, cy
  std::vector<double> intrinsics;
  // k1, k2, p1, p2, k3
  std::vector<double> distortion_coeffs;
};
}  // namespace slt_common
