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

#include "slt_common/cloud_registration/cloud_registration_factory.hpp"

#include "slt_common/cloud_registration/icp_registration.hpp"
#include "slt_common/cloud_registration/icp_svd_registration.hpp"
#include "slt_common/cloud_registration/ndt_registration.hpp"

namespace slt_common
{

CloudRegistrationFactory::CloudRegistrationFactory() {}

std::shared_ptr<CloudRegistrationInterface> CloudRegistrationFactory::create(
  const YAML::Node & config_node)
{
  auto registration_method = config_node["registration_method"].as<std::string>();
  std::shared_ptr<CloudRegistrationInterface> registration_ptr = nullptr;
  if (registration_method == "NDT") {
    registration_ptr = std::make_shared<NdtRegistration>(config_node["NDT"]);
  } else if (registration_method == "ICP") {
    registration_ptr = std::make_shared<IcpRegistration>(config_node["ICP"]);
  } else if (registration_method == "ICP_SVD") {
    registration_ptr = std::make_shared<IcpSvdRegistration>(config_node["ICP_SVD"]);
  } else {
    std::cerr << "Point cloud registration method " << registration_method << " NOT FOUND!";
  }
  return registration_ptr;
}

}  // namespace slt_common
