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

#include "slt_vio/feature_utils.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <map>

namespace slt_vio
{
bool has_enough_parallax(
  const std::map<int, FeatureObservation> & features,
  const std::map<int, FeatureObservation> & reference, double min_parallax)
{
  double parallax_sum = 0.0;
  int parallax_num = 0;
  for (const auto & id_pts : features) {
    const auto point = reference.find(id_pts.first);
    if (point == reference.end()) {
      continue;
    }
    const Eigen::Vector2d current_point = id_pts.second.xy;
    const Eigen::Vector2d reference_point = point->second.xy;
    const double dx = reference_point(0) - current_point(0);
    const double dy = reference_point(1) - current_point(1);
    parallax_sum += std::sqrt(dx * dx + dy * dy);
    parallax_num++;
  }
  if (parallax_num == 0) {
    return true;
  }
  return parallax_sum / parallax_num >= min_parallax;
}
}  // namespace slt_vio
