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

#include "slt_common/point_cloud_registration/pcl_ndt.hpp"

namespace slt_common
{

PclNdt::PclNdt(const YAML::Node & node)
: ndt_(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>())
{
  float res = node["res"].as<float>();
  float step_size = node["step_size"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();

  set_param(res, step_size, trans_eps, max_iter);
}

PclNdt::PclNdt(float res, float step_size, float trans_eps, int max_iter)
: ndt_(new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>())
{
  set_param(res, step_size, trans_eps, max_iter);
}

bool PclNdt::set_param(float res, float step_size, float trans_eps, int max_iter)
{
  ndt_->setResolution(res);
  ndt_->setStepSize(step_size);
  ndt_->setTransformationEpsilon(trans_eps);
  ndt_->setMaximumIterations(max_iter);

  return true;
}

bool PclNdt::set_target(const PointCloudPtr & target)
{
  ndt_->setInputTarget(target);
  return true;
}

bool PclNdt::match(
  const PclNdt::PointCloudPtr & input, const Eigen::Matrix4d & initial_pose)
{
  PointCloudPtr result_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  ndt_->setInputSource(input);
  ndt_->align(*result_cloud, initial_pose.cast<float>());
  return true;
}
Eigen::Matrix4d PclNdt::get_final_pose()
{
  return ndt_->getFinalTransformation().cast<double>();
}

double PclNdt::get_fitness_score() {return ndt_->getFitnessScore();}

void PclNdt::print_info()
{
  std::cout << "[NDT] "
            << "res: " << ndt_->getResolution() << ", "
            << "step_size: " << ndt_->getStepSize() << ", "
            << "trans_eps: " << ndt_->getTransformationEpsilon() << ", "
            << "max_iter: " << ndt_->getMaximumIterations() << std::endl;
}

}  // namespace slt_common
