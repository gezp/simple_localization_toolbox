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

#include <yaml-cpp/yaml.h>

#include <memory>

#include "slt_common/point_cloud_registration/point_cloud_registration_interface.hpp"

namespace slt_common
{
class PointCloudRegistrationFactory
{
public:
  PointCloudRegistrationFactory();
  std::shared_ptr<PointCloudRegistrationInterface> create(const YAML::Node & config_node);
};
}  // namespace slt_common
