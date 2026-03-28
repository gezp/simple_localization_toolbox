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

#include <chrono>
#include <string>
#include <vector>
#include <limits>
#include <unordered_map>

namespace slt_common
{

class TicToc
{
public:
  TicToc() = default;
  void tic();
  // elapsed time in ms
  double toc();

private:
  std::chrono::time_point<std::chrono::steady_clock> start_;
};

class AdvancedTicToc
{
public:
  struct TimeData
  {
    std::string label;
    int num_calls = 0;
    double total_elapsed_time = 0;
    double average_elapsed_time = 0;
    double min_elapsed_time = std::numeric_limits<double>::max();
    double max_elapsed_time = 0;
    double ema_elapsed_time = 0;
    std::chrono::time_point<std::chrono::steady_clock> start;
  };
  AdvancedTicToc() = default;
  void set_ema_alpha(double ema_alpha);
  void set_title(const std::string & title);
  void set_enable(bool enable);
  void add_label(const char * label);
  void tic(const char * label);
  void toc(const char * label, int output_step = 0);
  void print_info(const char * label);
  void print_all_info();
  // print all info triggered by num_calls of this label
  void print_all_info(const char * label, int output_step = 1);

private:
  std::unordered_map<std::string, TimeData> buffer_;
  std::vector<std::string> labels_;
  double ema_alpha_{0.01};
  std::string title_{"TicToc"};
  bool enable_{true};
};

}  // namespace slt_common
