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

#include "slt_common/point_cloud_registration/point_cloud_registration_factory.hpp"

#include "slt_common/point_cloud_registration/pcl_icp.hpp"
#include "slt_common/point_cloud_registration/icp_svd.hpp"
#include "slt_common/point_cloud_registration/pcl_ndt.hpp"
#include "slt_common/point_cloud_registration/ndt_omp.hpp"

namespace slt_common
{

PointCloudRegistrationFactory::PointCloudRegistrationFactory() {}

std::shared_ptr<PointCloudRegistrationInterface> PointCloudRegistrationFactory::create(
  const YAML::Node & config_node)
{
  auto method = config_node["method"].as<std::string>();
  std::shared_ptr<PointCloudRegistrationInterface> registration_ptr = nullptr;
  if (method == "PCL_NDT") {
    registration_ptr = std::make_shared<PclNdt>(config_node["PCL_NDT"]);
  } else if (method == "PCL_ICP") {
    registration_ptr = std::make_shared<PclIcp>(config_node["PCL_ICP"]);
  } else if (method == "ICP_SVD") {
    registration_ptr = std::make_shared<IcpSvd>(config_node["ICP_SVD"]);
  } else if (method == "NDT_OMP") {
    registration_ptr = std::make_shared<NdtOmp>(config_node["NDT_OMP"]);
  } else {
    std::cerr << "Point cloud registration method " << method << " NOT FOUND!";
  }
  return registration_ptr;
}

}  // namespace slt_common
