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

#include <deque>

#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"

namespace slt_eskf_locator
{

class ImuIntegration
{
public:
  ImuIntegration();
  ~ImuIntegration() {}
  bool init(
    const slt_common::ImuNavState & state, const slt_common::ImuData & imu_data);
  bool integrate(const slt_common::ImuData & imu_data);
  // set&get state
  void set_state(const slt_common::ImuNavState & state);
  const slt_common::ImuNavState & get_state();

private:
  // data
  std::deque<slt_common::ImuData> imu_data_buff_;
  slt_common::ImuNavState state_;
};

}  // namespace slt_eskf_locator
