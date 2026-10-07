// Copyright 2026 Zhenpeng Ge (https://github.com/gezp).
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

#include "slt_vio/initializer/global_sfm.hpp"

#include <array>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include "ceres/ceres.h"

#include "slt_common/math_utils.hpp"
#include "slt_vio/initializer/factor/reprojection_factor.hpp"

namespace slt_vio
{
namespace
{
// a feature of the structure from motion, position is the world position
struct SfmFeature
{
  bool state{false};
  std::vector<std::pair<int, Eigen::Vector2d>> observation;
  double position[3]{0.0, 0.0, 0.0};
};

// the frame major input as the feature major view triangulation and bundle read: a row per feature
// in id order, one observation per frame that saw it, oldest first
std::vector<SfmFeature> build_features(const std::vector<std::map<int, Eigen::Vector2d>> & frames)
{
  std::map<int, SfmFeature> by_id;
  for (int slot = 0; slot < static_cast<int>(frames.size()); slot++) {
    for (const auto & id_pts : frames[slot]) {
      by_id[id_pts.first].observation.emplace_back(slot, id_pts.second);
    }
  }
  std::vector<SfmFeature> sfm_f;
  sfm_f.reserve(by_id.size());
  for (const auto & feature : by_id) {
    sfm_f.push_back(feature.second);
  }
  return sfm_f;
}

// the world point of a pair of observations, false where the pair leaves no usable point
bool triangulate_point(
  const Eigen::Matrix4d & pose0, const Eigen::Matrix4d & pose1, const Eigen::Vector2d & point0,
  const Eigen::Vector2d & point1, Eigen::Vector3d & point_3d)
{
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * pose0.row(2) - pose0.row(0);
  design_matrix.row(1) = point0[1] * pose0.row(2) - pose0.row(1);
  design_matrix.row(2) = point1[0] * pose1.row(2) - pose1.row(0);
  design_matrix.row(3) = point1[1] * pose1.row(2) - pose1.row(1);
  const Eigen::Vector4d triangulated_point =
    design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  // the homogeneous divisor: a pair without a baseline leaves the point at infinity
  const double w = triangulated_point(3);
  if (!std::isfinite(w) || std::abs(w) < 1e-12) {
    return false;
  }
  point_3d = triangulated_point.head<3>() / w;
  return point_3d.allFinite();
}

bool solve_frame_by_pnp(
  Eigen::Matrix3d & r_initial, Eigen::Vector3d & p_initial, int frame_index,
  std::vector<SfmFeature> & sfm_f)
{
  std::vector<cv::Point2f> pts_2_vector;
  std::vector<cv::Point3f> pts_3_vector;
  for (auto & feature : sfm_f) {
    if (!feature.state) {
      continue;
    }
    for (const auto & obs : feature.observation) {
      if (obs.first == frame_index) {
        pts_2_vector.emplace_back(obs.second(0), obs.second(1));
        pts_3_vector.emplace_back(
          static_cast<float>(feature.position[0]), static_cast<float>(feature.position[1]),
          static_cast<float>(feature.position[2]));
        break;
      }
    }
  }
  if (pts_2_vector.size() < 10u) {
    return false;
  }
  cv::Mat r;
  cv::Mat rvec;
  cv::Mat t;
  cv::Mat tmp_r;
  cv::eigen2cv(r_initial, tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(p_initial, t);
  // the points are normalized, the camera every opencv step reads has a unit focal length
  if (!cv::solvePnP(pts_3_vector, pts_2_vector, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), rvec, t,
      true))
  {
    return false;
  }
  cv::Rodrigues(rvec, r);
  Eigen::MatrixXd r_pnp;
  Eigen::MatrixXd t_pnp;
  cv::cv2eigen(r, r_pnp);
  cv::cv2eigen(t, t_pnp);
  r_initial = r_pnp;
  p_initial = t_pnp;
  return true;
}

// relative pose of two frames from the essential matrix of their normalized correspondences
bool solve_relative_rt(
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & corres,
  Eigen::Matrix3d & rotation, Eigen::Vector3d & translation)
{
  // the ransac threshold of the relative pose: 0.3 pixel in the normalized image of a camera whose
  // focal length is 460 pixel
  constexpr double kRansacThreshold = 0.3 / 460.0;
  if (corres.size() < 15) {
    return false;
  }
  std::vector<cv::Point2f> ll;
  std::vector<cv::Point2f> rr;
  ll.reserve(corres.size());
  rr.reserve(corres.size());
  for (const auto & corr : corres) {
    ll.emplace_back(corr.first(0), corr.first(1));
    rr.emplace_back(corr.second(0), corr.second(1));
  }
  cv::Mat mask;
  const cv::Mat e =
    cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, kRansacThreshold, 0.99, mask);
  if (e.empty()) {
    return false;
  }
  cv::Mat rot;
  cv::Mat trans;
  const int inlier_count =
    cv::recoverPose(e, ll, rr, cv::Mat::eye(3, 3, CV_64F), rot, trans, mask);
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  for (int i = 0; i < 3; i++) {
    t(i) = trans.at<double>(i, 0);
    for (int j = 0; j < 3; j++) {
      r(i, j) = rot.at<double>(i, j);
    }
  }
  rotation = r.transpose();
  translation = -r.transpose() * t;
  return inlier_count > 12;
}

void triangulate_two_frames(
  int frame0, const Eigen::Matrix4d & pose0, int frame1, const Eigen::Matrix4d & pose1,
  std::vector<SfmFeature> & sfm_f)
{
  for (auto & feature : sfm_f) {
    if (feature.state) {
      continue;
    }
    bool has_0 = false;
    bool has_1 = false;
    Eigen::Vector2d point0;
    Eigen::Vector2d point1;
    for (const auto & obs : feature.observation) {
      if (obs.first == frame0) {
        point0 = obs.second;
        has_0 = true;
      }
      if (obs.first == frame1) {
        point1 = obs.second;
        has_1 = true;
      }
    }
    if (has_0 && has_1) {
      Eigen::Vector3d point_3d;
      // a pair that leaves no usable point keeps the feature for a later one
      if (!triangulate_point(pose0, pose1, point0, point1, point_3d)) {
        continue;
      }
      feature.state = true;
      feature.position[0] = point_3d(0);
      feature.position[1] = point_3d(1);
      feature.position[2] = point_3d(2);
    }
  }
}
}  // namespace

GlobalSfm::GlobalSfm(double min_init_parallax)
: min_init_parallax_(min_init_parallax) {}

bool GlobalSfm::find_seed_frame(
  const std::vector<std::map<int, Eigen::Vector2d>> & frames, int & seed_index,
  Eigen::Matrix4d & relative_pose) const
{
  // the oldest frame with enough correspondence and parallax to the newest one starts the sfm
  const std::map<int, Eigen::Vector2d> & right = frames.back();
  for (int i = 0; i + 1 < static_cast<int>(frames.size()); i++) {
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> corres;
    for (const auto & id_pts : right) {
      const auto it = frames[i].find(id_pts.first);
      if (it == frames[i].end()) {
        continue;
      }
      corres.emplace_back(it->second, id_pts.second);
    }
    if (corres.size() <= 20u) {
      continue;
    }
    double sum_parallax = 0.0;
    for (const auto & corr : corres) {
      sum_parallax += (corr.first - corr.second).norm();
    }
    if (sum_parallax / static_cast<int>(corres.size()) <= min_init_parallax_) {
      continue;
    }
    Eigen::Matrix3d relative_r;
    Eigen::Vector3d relative_t;
    if (solve_relative_rt(corres, relative_r, relative_t)) {
      seed_index = i;
      relative_pose = Eigen::Matrix4d::Identity();
      relative_pose.block<3, 3>(0, 0) = relative_r;
      relative_pose.block<3, 1>(0, 3) = relative_t;
      return true;
    }
  }
  return false;
}

bool GlobalSfm::reconstruct(
  const std::vector<std::map<int, Eigen::Vector2d>> & frames,
  std::vector<Eigen::Matrix4d> & frame_poses)
{
  const int frame_num = static_cast<int>(frames.size());
  int seed_index = 0;
  Eigen::Matrix4d relative_pose;
  if (!find_seed_frame(frames, seed_index, relative_pose)) {
    status_ = SfmStatus::kNoRelativePose;
    return false;
  }
  // the feature major view the triangulation and the bundle read
  std::vector<SfmFeature> sfm_f = build_features(frames);

  // the world to camera pose of every frame, the single representation the sfm works in
  std::vector<Eigen::Matrix4d> poses(frame_num, Eigen::Matrix4d::Identity());

  // the seed frame sits at the world origin, its world to camera pose the identity; it and the last
  // frame are the sfm's two starting views, the relative pose between them placing the last one
  const Eigen::Quaterniond last_rotation =
    Eigen::Quaterniond(relative_pose.block<3, 3>(0, 0)).inverse();
  poses[frame_num - 1].block<3, 3>(0, 0) = last_rotation.toRotationMatrix();
  poses[frame_num - 1].block<3, 1>(0, 3) = -1 *
    (poses[frame_num - 1].block<3, 3>(0, 0) * relative_pose.block<3, 1>(0, 3));

  // forward: solve the pose, then triangulate against the last frame
  for (int i = seed_index; i < frame_num - 1; i++) {
    if (i > seed_index) {
      Eigen::Matrix3d r_initial = poses[i - 1].block<3, 3>(0, 0);
      Eigen::Vector3d p_initial = poses[i - 1].block<3, 1>(0, 3);
      if (!solve_frame_by_pnp(r_initial, p_initial, i, sfm_f)) {
        status_ = SfmStatus::kPnpFailed;
        return false;
      }
      poses[i].block<3, 3>(0, 0) = r_initial;
      poses[i].block<3, 1>(0, 3) = p_initial;
    }
    triangulate_two_frames(i, poses[i], frame_num - 1, poses[frame_num - 1], sfm_f);
  }
  // triangulate the frames between the seed frame and the last one against it
  for (int i = seed_index + 1; i < frame_num - 1; i++) {
    triangulate_two_frames(seed_index, poses[seed_index], i, poses[i], sfm_f);
  }
  // backward: solve the pose from the next frame, then triangulate against the seed frame
  for (int i = seed_index - 1; i >= 0; i--) {
    Eigen::Matrix3d r_initial = poses[i + 1].block<3, 3>(0, 0);
    Eigen::Vector3d p_initial = poses[i + 1].block<3, 1>(0, 3);
    if (!solve_frame_by_pnp(r_initial, p_initial, i, sfm_f)) {
      status_ = SfmStatus::kPnpFailed;
      return false;
    }
    poses[i].block<3, 3>(0, 0) = r_initial;
    poses[i].block<3, 1>(0, 3) = p_initial;
    triangulate_two_frames(i, poses[i], seed_index, poses[seed_index], sfm_f);
  }
  // the features the two view chain never saw: use their first and last observation
  for (auto & feature : sfm_f) {
    if (feature.state || feature.observation.size() < 2) {
      continue;
    }
    const Eigen::Vector2d point0 = feature.observation.front().second;
    const Eigen::Vector2d point1 = feature.observation.back().second;
    Eigen::Vector3d point_3d;
    if (!triangulate_point(
        poses[feature.observation.front().first], poses[feature.observation.back().first], point0,
        point1, point_3d))
    {
      continue;
    }
    feature.state = true;
    feature.position[0] = point_3d(0);
    feature.position[1] = point_3d(1);
    feature.position[2] = point_3d(2);
  }

  // a full bundle adjustment over the sfm result: one array each for rotation and translation of
  // every pose. rotation is eigen's quaternion order, scalar last, as EigenQuaternionManifold reads
  std::vector<std::array<double, 4>> q_array(frame_num);
  std::vector<std::array<double, 3>> t_array(frame_num);
  ceres::Problem problem;
  // the problem owns what it is handed, it deletes a shared pointer only once
  auto * quaternion_manifold = new ceres::EigenQuaternionManifold();
  for (int i = 0; i < frame_num; i++) {
    t_array[i] = {poses[i](0, 3), poses[i](1, 3), poses[i](2, 3)};
    Eigen::Map<Eigen::Quaterniond>(q_array[i].data()) = poses[i].block<3, 3>(0, 0);
    problem.AddParameterBlock(q_array[i].data(), 4, quaternion_manifold);
    problem.AddParameterBlock(t_array[i].data(), 3);
    if (i == seed_index) {
      problem.SetParameterBlockConstant(q_array[i].data());
    }
    if (i == seed_index || i == frame_num - 1) {
      problem.SetParameterBlockConstant(t_array[i].data());
    }
  }
  for (auto & feature : sfm_f) {
    if (!feature.state) {
      continue;
    }
    for (const auto & obs : feature.observation) {
      ceres::CostFunction * cost_function =
        ReprojectionFactor::create(obs.second.x(), obs.second.y());
      problem.AddResidualBlock(
        cost_function, nullptr, q_array[obs.first].data(),
        t_array[obs.first].data(), feature.position);
    }
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_solver_time_in_seconds = 0.2;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (summary.termination_type == ceres::FAILURE ||
    (summary.termination_type == ceres::NO_CONVERGENCE && summary.final_cost >= 5e-03))
  {
    status_ = SfmStatus::kBaFailed;
    return false;
  }

  // the world poses the bundle settled on, packed as the camera poses of the window
  frame_poses.resize(frame_num);
  for (int i = 0; i < frame_num; i++) {
    // the world pose the bundle settled on is the inverse of the camera one it carries
    const Eigen::Map<const Eigen::Quaterniond> c_quat(q_array[i].data());
    const Eigen::Quaterniond world_rotation = c_quat.inverse();
    const Eigen::Vector3d world_translation = -1 * (world_rotation * Eigen::Vector3d(
        t_array[i][0], t_array[i][1], t_array[i][2]));
    frame_poses[i] = Eigen::Matrix4d::Identity();
    frame_poses[i].block<3, 3>(0, 0) = world_rotation.toRotationMatrix();
    frame_poses[i].block<3, 1>(0, 3) = world_translation;
  }
  status_ = SfmStatus::kOk;
  return true;
}

}  // namespace slt_vio
