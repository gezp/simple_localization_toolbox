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

#include "slt_common/loam/loam_registration.hpp"

#include <ceres/ceres.h>
#include <pcl/common/transforms.h>

#include "slt_common/loam/loam_factor.hpp"

// some references:
// https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/src/laserOdometry.cpp

namespace slt_common
{

using LoamEdgeCorrespondence = LoamRegistration::LoamEdgeCorrespondence;
using LoamPlanarCorrespondence = LoamRegistration::LoamPlanarCorrespondence;

double get_sqr_distance(const pcl::PointXYZL & p1, const pcl::PointXYZL & p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
         (p1.z - p2.z) * (p1.z - p2.z);
}

LoamRegistration::LoamRegistration(const YAML::Node & config)
{
  num_nearby_ring_ = config["correspondence_search"]["num_nearby_ring"].as<int>();
  double distance_threshold = config["correspondence_search"]["distance_threshold"].as<double>();
  sqr_distance_threshold_ = distance_threshold * distance_threshold;
  num_optimization_ = config["num_optimization"].as<int>();
  use_analytic_derivatives_ = config["use_analytic_derivatives"].as<bool>();
  ceres_loss_ = config["ceres_loss"].as<double>();
  max_num_iterations_ = config["max_num_iterations"].as<int>();
  min_num_residual_blocks_ = config["min_num_residual_blocks"].as<int>();
  minimizer_progress_to_stdout_ = config["minimizer_progress_to_stdout"].as<bool>();
  debug_ = config["debug"].as<bool>();
  bool enabel = config["enable_elapsed_time_statistics"].as<bool>();
  elapsed_time_statistics_.set_enable(enabel);
  elapsed_time_statistics_.set_title("LoamRegistration");
}

bool LoamRegistration::set_target(const LoamFeature & target)
{
  last_feature_ = target;
  last_edge_kd_tree_.reset(new pcl::KdTreeFLANN<PointType>);
  last_planar_kd_tree_.reset(new pcl::KdTreeFLANN<PointType>);
  last_edge_kd_tree_->setInputCloud(last_feature_.corner_less_sharp);
  last_planar_kd_tree_->setInputCloud(last_feature_.surface_less_flat);
  return true;
}

bool LoamRegistration::match(const LoamFeature & input, const Eigen::Matrix4d & initial_pose)
{
  elapsed_time_statistics_.tic("match");
  // initialize ceres parameters
  Eigen::Map<Eigen::Quaterniond> q(ceres_parameter_);
  Eigen::Map<Eigen::Vector3d> t(ceres_parameter_ + 4);
  q = Eigen::Quaterniond(initial_pose.block<3, 3>(0, 0));
  t = initial_pose.block<3, 1>(0, 3);
  // optimize
  for (int i = 0; i < num_optimization_; i++) {
    ceres::Problem problem;
    ceres::Manifold * q_manifold;
    if (use_analytic_derivatives_) {
      q_manifold = new SO3Manifold();
    } else {
      q_manifold = new ceres::EigenQuaternionManifold();
    }
    problem.AddParameterBlock(ceres_parameter_, 4);
    problem.AddParameterBlock(ceres_parameter_ + 4, 3);
    problem.SetManifold(ceres_parameter_, q_manifold);
    ceres::LossFunction * loss_function = new ceres::HuberLoss(ceres_loss_);
    // find correspondence for edge features
    elapsed_time_statistics_.tic("find edge correspondence");
    auto edge_infos = find_all_edge_correspondence(*input.corner_sharp, get_final_pose());
    // add residual blocks
    for (auto & info : edge_infos) {
      ceres::CostFunction * cost_function;
      if (use_analytic_derivatives_) {
        cost_function = new EdgeAnalyticFactor(info.current_point, info.last_p_j, info.last_p_l);
      } else {
        cost_function = EdgeFactor::create(info.current_point, info.last_p_j, info.last_p_l);
      }
      problem.AddResidualBlock(
        cost_function, loss_function, ceres_parameter_, ceres_parameter_ + 4);
    }
    elapsed_time_statistics_.toc("find edge correspondence");
    // find correspondence for planar features
    elapsed_time_statistics_.toc("find planar correspondence");
    auto planar_infos = find_all_planar_correspondence(*input.surface_flat, get_final_pose());
    // add residual blocks
    for (auto & info : planar_infos) {
      ceres::CostFunction * cost_function;
      if (use_analytic_derivatives_) {
        cost_function =
          new PlanarAnalyticFactor(info.current_point, info.last_p_j, info.last_p_l, info.last_p_m);
      } else {
        cost_function =
          PlanarFactor::create(info.current_point, info.last_p_j, info.last_p_l, info.last_p_m);
      }
      problem.AddResidualBlock(
        cost_function, loss_function, ceres_parameter_, ceres_parameter_ + 4);
    }
    elapsed_time_statistics_.toc("find planar correspondence");
    if (debug_) {
      std::cout << "optimization [" << i + 1 << "] edge points: " << edge_infos.size()
                << ", planar points: " << planar_infos.size() << std::endl;
    }
    if (problem.NumResidualBlocks() < min_num_residual_blocks_) {
      std::cout << "the num of residual blocks are too small: " << problem.NumResidualBlocks()
                << std::endl;
      return false;
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = max_num_iterations_;
    options.minimizer_progress_to_stdout = minimizer_progress_to_stdout_;
    ceres::Solver::Summary summary;
    elapsed_time_statistics_.tic("optimaztion");
    ceres::Solve(options, &problem, &summary);
    elapsed_time_statistics_.toc("optimaztion");
    // std::cout << summary.FullReport() << std::endl;
  }
  elapsed_time_statistics_.toc("match");
  elapsed_time_statistics_.print_all_info("match", 20);
  return true;
}

Eigen::Matrix4d LoamRegistration::get_final_pose()
{
  Eigen::Map<Eigen::Quaterniond> q(ceres_parameter_);
  Eigen::Map<Eigen::Vector3d> t(ceres_parameter_ + 4);
  Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
  q.normalize();
  final_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  final_pose.block<3, 1>(0, 3) = t;
  return final_pose;
}

std::vector<LoamEdgeCorrespondence> LoamRegistration::find_all_edge_correspondence(
  const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose)
{
  Eigen::Affine3f T_last_current(initial_pose.cast<float>());
  auto & last_edge_point_cloud = last_feature_.corner_less_sharp;
  std::vector<LoamEdgeCorrespondence> result;
  std::vector<int> search_indices;
  std::vector<float> search_sqr_distances;
  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    // transform point to last frame
    auto p = pcl::transformPoint(point_cloud.points[i], T_last_current);
    last_edge_kd_tree_->nearestKSearch(p, 1, search_indices, search_sqr_distances);
    // skip if the distance of closest_point is larger than threshold
    if (search_sqr_distances[0] > sqr_distance_threshold_) {
      continue;
    }
    int closest_point_idx = search_indices[0];
    int closest_point_ring_id = last_edge_point_cloud->points[closest_point_idx].label;
    int min_point_idx2 = -1;
    double min_point_sqr_distance2 = sqr_distance_threshold_;
    // search in the direction of increasing ring
    for (size_t j = closest_point_idx + 1; j < last_edge_point_cloud->points.size(); ++j) {
      int ring_id = last_edge_point_cloud->points[j].label;
      // if not in nearby rings, end the loop
      if (ring_id > (closest_point_ring_id + num_nearby_ring_)) {
        break;
      }
      // find point in nearby ring
      if (ring_id > closest_point_ring_id) {
        double sqr_distance = get_sqr_distance(last_edge_point_cloud->points[j], p);
        if (sqr_distance < min_point_sqr_distance2) {
          min_point_sqr_distance2 = sqr_distance;
          min_point_idx2 = j;
        }
      }
    }
    // search in the direction of decreasing ring
    for (int j = closest_point_idx - 1; j >= 0; --j) {
      int ring_id = last_edge_point_cloud->points[j].label;
      if (ring_id < (closest_point_ring_id + num_nearby_ring_)) {
        break;
      }
      if (ring_id < closest_point_ring_id) {
        double sqr_distance = get_sqr_distance(last_edge_point_cloud->points[j], p);
        if (sqr_distance < min_point_sqr_distance2) {
          min_point_sqr_distance2 = sqr_distance;
          min_point_idx2 = j;
        }
      }
    }
    // min_point_idx2 is valid
    if (min_point_idx2 >= 0) {
      LoamEdgeCorrespondence info;
      info.current_point = point_cloud.points[i].getVector3fMap().cast<double>();
      info.last_p_j =
        last_edge_point_cloud->points[closest_point_idx].getVector3fMap().cast<double>();
      info.last_p_l = last_edge_point_cloud->points[min_point_idx2].getVector3fMap().cast<double>();
      result.push_back(std::move(info));
    }
  }
  return result;
}

std::vector<LoamPlanarCorrespondence> LoamRegistration::find_all_planar_correspondence(
  const PointCloudType & point_cloud, const Eigen::Matrix4d & initial_pose)
{
  Eigen::Affine3f T_last_current(initial_pose.cast<float>());

  auto & last_planar_point_cloud = last_feature_.surface_less_flat;
  std::vector<LoamPlanarCorrespondence> result;
  std::vector<int> search_indices;
  std::vector<float> search_sqr_distances;
  for (size_t i = 0; i < point_cloud.points.size(); ++i) {
    // transform point to last frame
    auto p = pcl::transformPoint(point_cloud.points[i], T_last_current);
    last_planar_kd_tree_->nearestKSearch(p, 1, search_indices, search_sqr_distances);
    // skip if the distance of closest_point is larger than threshold
    if (search_sqr_distances[0] > sqr_distance_threshold_) {
      continue;
    }
    int closest_point_idx = search_indices[0];
    int closest_point_ring_id = last_planar_point_cloud->points[closest_point_idx].label;
    int min_point_idx2 = -1;
    double min_point_sqr_distance2 = sqr_distance_threshold_;
    int min_point_idx3 = -1;
    double min_point_sqr_distance3 = sqr_distance_threshold_;
    // search in the direction of increasing ring
    for (size_t j = closest_point_idx + 1; j < last_planar_point_cloud->points.size(); j++) {
      int ring_id = last_planar_point_cloud->points[j].label;
      // if not in nearby rings, end the loop
      if (ring_id > (closest_point_ring_id + num_nearby_ring_)) {
        break;
      }
      double sqr_distance = get_sqr_distance(last_planar_point_cloud->points[j], p);
      if (ring_id <= closest_point_ring_id && sqr_distance < min_point_sqr_distance2) {
        // find the closest point in current ring
        min_point_sqr_distance2 = sqr_distance;
        min_point_idx2 = j;
      } else if (ring_id > closest_point_ring_id && sqr_distance < min_point_sqr_distance3) {
        // find the closest point in nearby ring
        min_point_sqr_distance3 = sqr_distance;
        min_point_idx3 = j;
      }
    }
    // search in the direction of decreasing ring
    for (int j = closest_point_idx - 1; j >= 0; --j) {
      int ring_id = last_planar_point_cloud->points[j].label;
      if (ring_id < (closest_point_ring_id - num_nearby_ring_)) {
        break;
      }
      double sqr_distance = get_sqr_distance(last_planar_point_cloud->points[j], p);
      if (ring_id >= closest_point_ring_id && sqr_distance < min_point_sqr_distance2) {
        min_point_sqr_distance2 = sqr_distance;
        min_point_idx2 = j;
      } else if (ring_id < closest_point_idx && sqr_distance < min_point_sqr_distance3) {
        min_point_sqr_distance3 = sqr_distance;
        min_point_idx3 = j;
      }
    }
    // valid result
    if (min_point_idx2 >= 0 && min_point_idx3 >= 0) {
      LoamPlanarCorrespondence info;
      info.current_point = point_cloud.points[i].getVector3fMap().cast<double>();
      info.last_p_j =
        last_planar_point_cloud->points[closest_point_idx].getVector3fMap().cast<double>();
      info.last_p_l =
        last_planar_point_cloud->points[min_point_idx2].getVector3fMap().cast<double>();
      info.last_p_m =
        last_planar_point_cloud->points[min_point_idx3].getVector3fMap().cast<double>();
      result.push_back(std::move(info));
    }
  }
  return result;
}

}  // namespace slt_common
