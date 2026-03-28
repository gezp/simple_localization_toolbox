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

#include "slt_common/tic_toc.hpp"

#include <iostream>

namespace slt_common
{
// some references:
// https://github.com/RainerKuemmerle/g2o/blob/master/g2o/stuff/tictoc.cpp

void TicToc::tic()
{
  start_ = std::chrono::steady_clock::now();
}

double TicToc::toc()
{
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start_;
  return elapsed_seconds.count() * 1000;
}

void AdvancedTicToc::set_ema_alpha(double ema_alpha)
{
  ema_alpha_ = ema_alpha;
}

void AdvancedTicToc::set_enable(bool enable)
{
  enable_ = enable;
}

void AdvancedTicToc::set_title(const std::string & title)
{
  title_ = title;
}

void AdvancedTicToc::add_label(const char * label)
{
  TimeData data;
  data.label = label;
  buffer_[data.label] = data;
  labels_.push_back(data.label);
}

void AdvancedTicToc::tic(const char * label)
{
  if (!enable_) {
    return;
  }
  auto it = buffer_.find(label);
  if (it == buffer_.end()) {
    TimeData data;
    data.label = label;
    data.start = std::chrono::steady_clock::now();
    buffer_[data.label] = data;
    labels_.push_back(data.label);
  } else {
    it->second.start = std::chrono::steady_clock::now();
  }
}

void AdvancedTicToc::toc(const char * label, int output_step)
{
  if (!enable_) {
    return;
  }
  auto it = buffer_.find(label);
  if (it == buffer_.end()) {
    return;
  }
  // get elapsed time
  auto & data = it->second;
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - data.start;
  double elapsed_ms = elapsed_seconds.count() * 1000;
  // update statistics
  data.num_calls++;
  data.total_elapsed_time += elapsed_ms;
  data.average_elapsed_time = data.total_elapsed_time / data.num_calls;
  data.min_elapsed_time = std::min(data.min_elapsed_time, elapsed_ms);
  data.max_elapsed_time = std::max(data.max_elapsed_time, elapsed_ms);
  data.ema_elapsed_time = (1 - ema_alpha_) * data.ema_elapsed_time + ema_alpha_ * elapsed_ms;
  if (output_step > 0 && data.num_calls % output_step == 0) {
    std::cout << "[" << title_ << "][" << data.label << "] num calls=" << data.num_calls
              << ", cur=" << elapsed_ms << "ms, avg=" << data.average_elapsed_time
              << "ms, min=" << data.min_elapsed_time << "ms, max=" << data.max_elapsed_time
              << "ms, ema=" << data.ema_elapsed_time << "ms" << std::endl;
  }
}

void AdvancedTicToc::print_info(const char * label)
{
  auto it = buffer_.find(label);
  if (it != buffer_.end()) {
    auto & data = it->second;
    std::cout << "[" << title_ << "][" << data.label << "] num calls=" << data.num_calls
              << ", avg=" << data.average_elapsed_time << "ms, min=" << data.min_elapsed_time
              << "ms, max=" << data.max_elapsed_time << "ms, ema=" << data.ema_elapsed_time << "ms"
              << std::endl;
  }
}

void AdvancedTicToc::print_all_info()
{
  if (!enable_) {
    return;
  }
  for (auto & label : labels_) {
    print_info(label.c_str());
  }
}

void AdvancedTicToc::print_all_info(const char * label, int output_step)
{
  if (!enable_) {
    return;
  }
  auto it = buffer_.find(label);
  if (it == buffer_.end()) {
    return;
  }
  auto & data = it->second;
  if (output_step > 0 && data.num_calls % output_step == 0) {
    for (auto & label : labels_) {
      print_info(label.c_str());
    }
  }
}

}  // namespace slt_common
