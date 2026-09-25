// Copyright 2026 Zhenpeng Ge (https://github.com/zhenpengge).
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

#include <pcl/kdtree/kdtree_flann.h>

#include "slt_lio/kalman_filter/eskf.hpp"

namespace slt_lio
{

// point-to-plane residual helpers shared by fastlio & fastlio2.
// r = n' * (p_world - q), p_world = R * p_body + t, (q, n) the matched plane from the
// local map. analytic jacobian w.r.t. the right-perturbation error state:
//   dr/dx_pos = n', dr/dx_ori = -n' * R * [p_body]_x
struct PointPlaneResidual
{
  // compute residual & 1x6 jacobian [pos, ori] at linearization point (R, t)
  static double evaluate(
    const Eigen::Vector3d & p_body, const Eigen::Vector3d & plane_point,
    const Eigen::Vector3d & plane_normal, const Eigen::Matrix3d & R, const Eigen::Vector3d & t,
    Eigen::Matrix<double, 1, 6> & jacobian)
  {
    Eigen::Vector3d p_world = R * p_body + t;
    double residual = plane_normal.dot(p_world - plane_point);
    jacobian.block<1, 3>(0, 0) = plane_normal.transpose();
    jacobian.block<1, 3>(0, 3) = -plane_normal.transpose() * R * Sophus::SO3d::hat(p_body);
    return residual;
  }
};

// plane fit from neighbor points: smallest eigen vector of the covariance
struct PlaneFit
{
  static bool fit(
    const std::vector<Eigen::Vector3d> & neighbors, double eigen_ratio,
    Eigen::Vector3d & plane_point, Eigen::Vector3d & plane_normal)
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (auto & p : neighbors) {
      center += p;
    }
    center /= static_cast<double>(neighbors.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (auto & p : neighbors) {
      Eigen::Vector3d q = p - center;
      covariance += q * q.transpose();
    }
    covariance /= static_cast<double>(neighbors.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    plane_normal = solver.eigenvectors().col(0);
    double smallest = solver.eigenvalues()(0);
    double middle = solver.eigenvalues()(1);
    if (smallest <= std::numeric_limits<double>::epsilon() || middle < eigen_ratio * smallest) {
      return false;
    }
    plane_point = center;
    return true;
  }
};

// point-to-plane correspondences against a pcl kdtree map (fastlio): nearest
// neighbors + plane fit
class PointPlaneCorrespondenceSearch
{
  using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

public:
  struct Config
  {
    int num_nearby{5};
    double nearby_distance{1.0};
    double plane_eigen_ratio{3.0};
    double max_plane_distance{0.2};
  };

  static std::vector<Eskf::PlaneObservation> search(
    const PointCloudPtr & input, const PointCloudPtr & map,
    const pcl::KdTreeFLANN<pcl::PointXYZ> & kd_tree, const Eigen::Matrix3d & R,
    const Eigen::Vector3d & t, const Config & config)
  {
    std::vector<Eskf::PlaneObservation> observations;
    if (!map || map->empty() || kd_tree.getInputCloud() == nullptr) {
      return observations;
    }
    double sqr_nearby_distance = config.nearby_distance * config.nearby_distance;
    std::vector<int> search_indices;
    std::vector<float> search_sqr_distances;
    for (size_t i = 0; i < input->points.size(); ++i) {
      Eigen::Vector3d p_body = input->points[i].getVector3fMap().cast<double>();
      Eigen::Vector3d p_world = R * p_body + t;
      pcl::PointXYZ p_map;
      p_map.getVector3fMap() = p_world.cast<float>();
      kd_tree.nearestKSearch(p_map, config.num_nearby, search_indices, search_sqr_distances);
      if (static_cast<int>(search_indices.size()) < config.num_nearby) {
        continue;
      }
      if (search_sqr_distances.back() > sqr_nearby_distance) {
        continue;
      }
      std::vector<Eigen::Vector3d> neighbors;
      neighbors.reserve(search_indices.size());
      for (auto idx : search_indices) {
        neighbors.push_back(map->points[idx].getVector3fMap().cast<double>());
      }
      Eigen::Vector3d plane_point, plane_normal;
      if (!PlaneFit::fit(neighbors, config.plane_eigen_ratio, plane_point, plane_normal)) {
        continue;
      }
      if (std::abs((p_world - plane_point).dot(plane_normal)) > config.max_plane_distance) {
        continue;
      }
      Eskf::PlaneObservation observation;
      observation.point = p_body;
      observation.plane_point = plane_point;
      observation.plane_normal = plane_normal;
      observations.push_back(observation);
    }
    return observations;
  }
};

}  // namespace slt_lio
