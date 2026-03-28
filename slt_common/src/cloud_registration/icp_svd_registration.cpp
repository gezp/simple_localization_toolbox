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

#include "slt_common/cloud_registration/icp_svd_registration.hpp"

#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <cmath>
#include <vector>

namespace slt_common
{

IcpSvdRegistration::IcpSvdRegistration(const YAML::Node & node)
: input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
{
  // parse params:
  float max_corr_dist = node["max_corr_dist"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  float euc_fitness_eps = node["euc_fitness_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();

  set_param(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

IcpSvdRegistration::IcpSvdRegistration(
  float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter)
: input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>())
{
  set_param(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool IcpSvdRegistration::set_param(
  float max_corr_dist, float trans_eps, float euc_fitness_eps, int max_iter)
{
  // set params:
  max_corr_dist_ = max_corr_dist;
  trans_eps_ = trans_eps;
  euc_fitness_eps_ = euc_fitness_eps;
  max_iter_ = max_iter;

  return true;
}

bool IcpSvdRegistration::set_target(const IcpSvdRegistration::PointCloudPtr & target)
{
  input_target_ = target;
  input_target_kdtree_->setInputCloud(input_target_);
  return true;
}

bool IcpSvdRegistration::match(
  const IcpSvdRegistration::PointCloudPtr & input, const Eigen::Matrix4d & initial_pose)
{
  input_source_ = input;

  // pre-process input source:
  PointCloudPtr transformed_input_source(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*input_source_, *transformed_input_source, initial_pose);

  // init estimation:
  transformation_.setIdentity();
  Eigen::Matrix4f dT;
  dT.setIdentity();
  std::vector<Eigen::Vector3f> xs, ys;
  // do estimation:
  int curr_iter = 0;
  while (curr_iter < max_iter_) {
    // apply current estimation:
    pcl::transformPointCloud(*transformed_input_source, *transformed_input_source, dT);
    // get correspondence:
    auto num_corr = get_correspondence(transformed_input_source, xs, ys);
    // do not have enough correspondence -- break:
    if (num_corr < 3) {
      break;
    }
    // update current transform:
    get_transform(xs, ys, dT);
    // whether the transformation update is significant:
    if (!is_significant(dT, trans_eps_)) {
      break;
    }
    // update transformation:
    transformation_ = dT * transformation_;
    ++curr_iter;
  }
  // set output:
  final_pose_ = transformation_.cast<double>() * initial_pose;
  return true;
}
Eigen::Matrix4d IcpSvdRegistration::get_final_pose() {return final_pose_;}

double IcpSvdRegistration::get_fitness_score() {return 1.0;}

void IcpSvdRegistration::print_info()
{
  std::cout << "[ICP_SVD] "
            << "max_corr_dist: " << max_corr_dist_ << ", "
            << "trans_eps: " << trans_eps_ << ", "
            << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
            << "max_iter: " << max_iter_ << std::endl;
}

size_t IcpSvdRegistration::get_correspondence(
  const IcpSvdRegistration::PointCloudPtr & input_source, std::vector<Eigen::Vector3f> & xs,
  std::vector<Eigen::Vector3f> & ys)
{
  const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

  size_t num_corr = 0;
  // set up point correspondence
  pcl::Indices ids;
  std::vector<float> distances;
  int found_num;
  for (size_t i = 0; i < input_source->size(); i++) {
    auto py = input_source->at(i);
    if (!pcl::isFinite(py)) {
      continue;
    }
    found_num = input_target_kdtree_->nearestKSearch(py, 1, ids, distances);
    if (found_num == 1 && distances[0] < MAX_CORR_DIST_SQR) {
      auto px = input_target_->at(ids[0]);
      xs.push_back({px.x, px.y, px.z});
      ys.push_back({py.x, py.y, py.z});
      num_corr++;
    }
  }
  return num_corr;
}

void IcpSvdRegistration::get_transform(
  const std::vector<Eigen::Vector3f> & xs, const std::vector<Eigen::Vector3f> & ys,
  Eigen::Matrix4f & transformation)
{
  const size_t N = xs.size();
  assert(xs.size() == ys.size());
  // find centroids of mu_x and mu_y:
  Eigen::Vector3f mu_x(0, 0, 0), mu_y(0, 0, 0);
  for (size_t i = 0; i < N; i++) {
    mu_x += xs[i];
    mu_y += ys[i];
  }
  mu_x /= N;
  mu_y /= N;
  // build H
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (size_t i = 0; i < N; i++) {
    H += (ys[i] - mu_y) * (xs[i] - mu_x).transpose();
  }
  // solve R
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  auto V = svd.matrixV();
  auto U = svd.matrixU();
  auto R = V * U.transpose();
  // solve t
  auto t = mu_x - R * mu_y;
  // set output
  transformation.setIdentity();
  transformation.block<3, 3>(0, 0) = R;
  transformation.block<3, 1>(0, 3) = t;
}

bool IcpSvdRegistration::is_significant(
  const Eigen::Matrix4f & transformation, const float trans_eps)
{
  // a. translation magnitude -- norm:
  float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
  // b. rotation magnitude -- angle:
  float rotation_magnitude = fabs(acos((transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f));

  return (translation_magnitude > trans_eps) || (rotation_magnitude > trans_eps);
}

}  // namespace slt_common
