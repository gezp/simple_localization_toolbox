// Copyright 2023 Gezp (https://github.com/gezp).
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

#include "slt_common/sensor_data/lidar_data.hpp"
#include "slt_common/sensor_data/twist_data.hpp"

namespace slt_common
{

bool convert_velodyne64(LidarData & lidar_data, double dt = 0.1, bool is_clockwise = false);

bool undistort_point_cloud(LidarData & lidar_data, const TwistData & twist_data);

}  // namespace slt_common
