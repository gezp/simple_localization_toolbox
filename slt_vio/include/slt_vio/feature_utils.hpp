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

#include <map>

#include "slt_vio/feature.hpp"

namespace slt_vio
{
// whether the mean parallax of the features two frames share is at or above the threshold, in the
// normalized image: features the reference lacks are left out, and sharing none is enough
bool has_enough_parallax(
  const std::map<int, FeatureObservation> & features,
  const std::map<int, FeatureObservation> & reference, double min_parallax);
}  // namespace slt_vio
