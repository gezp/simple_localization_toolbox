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

#include "localization_common/odom_data_buffer.hpp"

#include "localization_common/sensor_data_utils.hpp"

namespace localization_common
{

OdomDataBuffer::OdomDataBuffer(size_t max_buffer_size) {max_buffer_size_ = max_buffer_size;}

void OdomDataBuffer::add_data(const OdomData & data)
{
  buffer_.insert({data.time, data});
  if (buffer_.size() > max_buffer_size_) {
    buffer_.erase(buffer_.begin());
  }
}

bool OdomDataBuffer::get_data(double time, OdomData & data)
{
  if (auto it = buffer_.find(time); it != buffer_.end()) {
    data = it->second;
    return true;
  }
  return false;
}

bool OdomDataBuffer::get_nearest_data(double time, OdomData & data)
{
  if (buffer_.empty()) {
    return false;
  }
  auto cur = buffer_.lower_bound(time);
  if (cur == buffer_.end()) {
    // the last
    data = buffer_.rbegin()->second;
  } else if (cur == buffer_.begin()) {
    // the first
    data = cur->second;
  } else {
    // choose between prev and cur
    auto prev = std::prev(cur);
    if (fabs(prev->first - time) < fabs(cur->first - time)) {
      data = prev->second;
    } else {
      data = cur->second;
    }
  }
  return true;
}

bool OdomDataBuffer::get_interpolated_data(double time, OdomData & data)
{
  if (buffer_.empty()) {
    return false;
  }
  if (time < buffer_.begin()->first || time > buffer_.rbegin()->first) {
    return false;
  }
  auto cur = buffer_.lower_bound(time);
  if (cur == buffer_.end()) {
    return false;
  } else if (cur == buffer_.begin()) {
    // the first
    data = cur->second;
  } else {
    // interpolate between prev and cur
    auto prev = std::prev(cur);
    data = interpolate_odom(prev->second, cur->second, time);
  }
  return true;
}

std::vector<OdomData> OdomDataBuffer::get_vector()
{
  std::vector<OdomData> v;
  for (auto & kv : buffer_) {
    v.push_back(kv.second);
  }
  return v;
}

double OdomDataBuffer::get_start_time()
{
  if (buffer_.empty()) {
    return -1;
  }
  return buffer_.begin()->first;
}

double OdomDataBuffer::get_end_time()
{
  if (buffer_.empty()) {
    return -1;
  }
  return buffer_.rbegin()->first;
}

void OdomDataBuffer::remove(double time)
{
  if (auto it = buffer_.find(time); it != buffer_.end()) {
    buffer_.erase(it);
  }
}

void OdomDataBuffer::remove_before(double time)
{
  auto cur = buffer_.lower_bound(time);
  if (cur != buffer_.begin()) {
    cur--;
    buffer_.erase(buffer_.begin(), cur);
  }
}

void OdomDataBuffer::remove_after(double time)
{
  auto cur = buffer_.upper_bound(time);
  buffer_.erase(cur, buffer_.end());
}

size_t OdomDataBuffer::size() {return buffer_.size();}

void OdomDataBuffer::set_max_buffer_size(size_t max_buffer_size)
{
  max_buffer_size_ = max_buffer_size;
}

}  // namespace localization_common
