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

#include "slt_common/point_cloud_registration/ndt_omp.hpp"

namespace slt_common
{

NdtOmp::NdtOmp(const YAML::Node & node)
: ndt_(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>())
{
  float res = node["res"].as<float>();
  float step_size = node["step_size"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();
  if (node["thread_num"]) {
    thread_num_ = node["thread_num"].as<int>();
  }
  set_param(res, step_size, trans_eps, max_iter);
}

NdtOmp::NdtOmp(float res, float step_size, float trans_eps, int max_iter)
: ndt_(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>())
{
  set_param(res, step_size, trans_eps, max_iter);
}

bool NdtOmp::set_param(float res, float step_size, float trans_eps, int max_iter)
{
  ndt_->setResolution(res);
  ndt_->setStepSize(step_size);
  ndt_->setTransformationEpsilon(trans_eps);
  ndt_->setMaximumIterations(max_iter);
  ndt_->setNumThreads(thread_num_);
  return true;
}

bool NdtOmp::set_target(const PointCloudPtr & target)
{
  ndt_->setInputTarget(target);
  return true;
}

bool NdtOmp::match(
  const NdtOmp::PointCloudPtr & input, const Eigen::Matrix4d & initial_pose)
{
  PointCloudPtr result_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  ndt_->setInputSource(input);
  ndt_->align(*result_cloud, initial_pose.cast<float>());
  return true;
}
Eigen::Matrix4d NdtOmp::get_final_pose()
{
  return ndt_->getFinalTransformation().cast<double>();
}

double NdtOmp::get_fitness_score() {return ndt_->getFitnessScore();}

void NdtOmp::print_info()
{
  std::cout << "[NDT_OMP] "
            << "res: " << ndt_->getResolution() << ", "
            << "step_size: " << ndt_->getStepSize() << ", "
            << "trans_eps: " << ndt_->getTransformationEpsilon() << ", "
            << "max_iter: " << ndt_->getMaximumIterations() << ", "
            << "thread_num: " << thread_num_ << std::endl;
}

}  // namespace slt_common
