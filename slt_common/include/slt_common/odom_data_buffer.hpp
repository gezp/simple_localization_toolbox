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

#include <map>
#include <vector>

#include "slt_common/sensor_data/odom_data.hpp"

namespace slt_common
{
class OdomDataBuffer
{
public:
  explicit OdomDataBuffer(size_t max_buffer_size = 1000000);
  ~OdomDataBuffer() = default;
  // add
  void add_data(const OdomData & data);
  // get
  bool get_data(double time, OdomData & data);
  bool get_nearest_data(double time, OdomData & data);
  bool get_interpolated_data(double time, OdomData & data);
  std::vector<OdomData> get_vector();
  double get_start_time();
  double get_end_time();
  // remove
  void remove(double time);
  void remove_before(double time);
  void remove_after(double time);
  // size
  size_t size();
  void set_max_buffer_size(size_t max_buffer_size);

private:
  std::map<int64_t, slt_common::OdomData> buffer_;
  size_t max_buffer_size_;
};
}  // namespace slt_common
