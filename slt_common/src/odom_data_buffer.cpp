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

#include "slt_common/odom_data_buffer.hpp"

#include "slt_common/sensor_data_utils.hpp"

namespace slt_common
{

OdomDataBuffer::OdomDataBuffer(size_t max_buffer_size) {max_buffer_size_ = max_buffer_size;}

void OdomDataBuffer::add_data(const OdomData & data)
{
  buffer_.insert({static_cast<int64_t>(data.time * 1000000), data});
  if (buffer_.size() > max_buffer_size_) {
    buffer_.erase(buffer_.begin());
  }
}

bool OdomDataBuffer::get_data(double time, OdomData & data)
{
  if (auto it = buffer_.find(static_cast<int64_t>(time * 1000000)); it != buffer_.end()) {
    data = it->second;
    return true;
  }
  return false;
}

bool OdomDataBuffer::get_data_at_or_after(double time, OdomData & data)
{
  auto it = buffer_.lower_bound(static_cast<int64_t>(time * 1000000));
  if (it == buffer_.end()) {
    return false;
  }
  data = it->second;
  return true;
}

bool OdomDataBuffer::get_data_after(double time, OdomData & data)
{
  auto it = buffer_.upper_bound(static_cast<int64_t>(time * 1000000));
  if (it == buffer_.end()) {
    return false;
  }
  data = it->second;
  return true;
}

bool OdomDataBuffer::get_data_at_or_before(double time, OdomData & data)
{
  auto it = buffer_.upper_bound(static_cast<int64_t>(time * 1000000));
  if (it == buffer_.begin()) {
    return false;
  }
  data = std::prev(it)->second;
  return true;
}

bool OdomDataBuffer::get_data_before(double time, OdomData & data)
{
  auto it = buffer_.lower_bound(static_cast<int64_t>(time * 1000000));
  if (it == buffer_.begin()) {
    return false;
  }
  data = std::prev(it)->second;
  return true;
}

bool OdomDataBuffer::get_nearest_data(double time, OdomData & data)
{
  if (buffer_.empty()) {
    return false;
  }
  int64_t time_us = static_cast<int64_t>(time * 1000000);
  auto cur = buffer_.lower_bound(time_us);
  if (cur == buffer_.end()) {
    // the last
    data = buffer_.rbegin()->second;
  } else if (cur == buffer_.begin()) {
    // the first
    data = cur->second;
  } else {
    // choose between prev and cur
    auto prev = std::prev(cur);
    if (std::abs(prev->first - time_us) < std::abs(cur->first - time_us)) {
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
  int64_t time_us = static_cast<int64_t>(time * 1000000);
  if (time_us < buffer_.begin()->first || time_us > buffer_.rbegin()->first) {
    return false;
  }
  auto cur = buffer_.lower_bound(time_us);
  if (cur == buffer_.end()) {
    return false;
  }
  if (cur->first == time_us) {
    // exact match, no need to interpolate
    data = cur->second;
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
  return static_cast<double>(buffer_.begin()->first) / 1000000.0;
}

double OdomDataBuffer::get_end_time()
{
  if (buffer_.empty()) {
    return -1;
  }
  return static_cast<double>(buffer_.rbegin()->first) / 1000000.0;
}

void OdomDataBuffer::remove(double time)
{
  if (auto it = buffer_.find(static_cast<int64_t>(time * 1000000)); it != buffer_.end()) {
    buffer_.erase(it);
  }
}

void OdomDataBuffer::remove_before(double time)
{
  auto cur = buffer_.lower_bound(static_cast<int64_t>(time * 1000000));
  if (cur != buffer_.begin()) {
    cur--;
    buffer_.erase(buffer_.begin(), cur);
  }
}

void OdomDataBuffer::remove_after(double time)
{
  auto cur = buffer_.upper_bound(static_cast<int64_t>(time * 1000000));
  buffer_.erase(cur, buffer_.end());
}

size_t OdomDataBuffer::size() {return buffer_.size();}

void OdomDataBuffer::set_max_buffer_size(size_t max_buffer_size)
{
  max_buffer_size_ = max_buffer_size;
}

}  // namespace slt_common
