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

#include "slt_lio/kalman_filter/eskf.hpp"

namespace slt_lio
{

Eskf::Eskf(const YAML::Node & config)
{
  // process noise
  noise_prior_ = config["covariance"]["prior"].as<double>();
  noise_prior_gravity_ = config["covariance"]["prior_gravity"].as<double>();
  noise_accel_ = config["covariance"]["accel"].as<double>();
  noise_gyro_ = config["covariance"]["gyro"].as<double>();
  noise_accel_bias_ = config["covariance"]["accel_bias"].as<double>();
  noise_gyro_bias_ = config["covariance"]["gyro_bias"].as<double>();
  Q_ = Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise>::Zero();
  // observation
  max_num_iterations_ = config["max_num_iterations"].as<int>();
  converge_threshold_ = config["converge_threshold"].as<double>();
  observation_noise_ = config["observation_noise"].as<double>();
  P_ = Eigen::Matrix<double, kDimState, kDimState>::Zero();
}

void Eskf::init_state(const slt_common::ImuNavState & state, const slt_common::ImuData & imu_data)
{
  imu_integration_.reset(state, true);
  imu_integration_.integrate(imu_data);
  P_ = Eigen::Matrix<double, kDimState, kDimState>::Identity() * noise_prior_;
  P_.block<3, 3>(kIndexErrorGravity, kIndexErrorGravity) =
    Eigen::Matrix3d::Identity() * noise_prior_gravity_;
  X_ = Eigen::Matrix<double, kDimState, 1>::Zero();
  is_inited_ = true;
}

bool Eskf::predict(const slt_common::ImuData & imu_data)
{
  if (!is_inited_) {
    return false;
  }
  const auto & state = imu_integration_.get_imu_nav_state();
  double dt = imu_data.time - state.time;
  if (dt < 0.0) {
    return false;
  }
  const Eigen::Matrix3d & R = state.orientation;
  const Eigen::Vector3d w_b = imu_data.angular_velocity - state.gyro_bias;
  const Eigen::Vector3d a_b = imu_data.linear_acceleration - state.accel_bias;
  // propagate nominal state by midpoint integration
  imu_integration_.integrate(imu_data);
  // propagate covariance: X = F * X + B * w
  Eigen::Matrix3d R_mid = R * Sophus::SO3d::exp(0.5 * w_b * dt).matrix();
  F_ = Eigen::Matrix<double, kDimState, kDimState>::Identity();
  F_.block<3, 3>(kIndexErrorPos, kIndexErrorVel) = Eigen::Matrix3d::Identity() * dt;
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorOri) = Sophus::SO3d::exp(-w_b * dt).matrix();
  F_.block<3, 3>(kIndexErrorOri, kIndexErrorGyro) = -Sophus::SO3d::exp(-w_b * dt).matrix() * dt;
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorOri) = -R_mid * Sophus::SO3d::hat(a_b) * dt;
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorAccel) = -R_mid * dt;
  F_.block<3, 3>(kIndexErrorVel, kIndexErrorGravity) = Eigen::Matrix3d::Identity() * dt;
  B_ = Eigen::Matrix<double, kDimState, kDimProcessNoise>::Zero();
  B_.block<3, 3>(kIndexErrorOri, kIndexNoiseGyro) = R_mid * dt;
  B_.block<3, 3>(kIndexErrorVel, kIndexNoiseAccel) = R_mid * dt;
  B_.block<3, 3>(kIndexErrorAccel, kIndexNoiseBiasAccel) =
    Eigen::Matrix3d::Identity() * std::sqrt(dt);
  B_.block<3, 3>(kIndexErrorGyro, kIndexNoiseBiasGyro) =
    Eigen::Matrix3d::Identity() * std::sqrt(dt);
  // noise matrix (config values are variances, same as slt_eskf_locator)
  Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise> Q =
    Eigen::Matrix<double, kDimProcessNoise, kDimProcessNoise>::Zero();
  Q.block<3, 3>(kIndexNoiseAccel, kIndexNoiseAccel) = Eigen::Matrix3d::Identity() * noise_accel_;
  Q.block<3, 3>(kIndexNoiseGyro, kIndexNoiseGyro) = Eigen::Matrix3d::Identity() * noise_gyro_;
  Q.block<3, 3>(kIndexNoiseBiasAccel, kIndexNoiseBiasAccel) =
    Eigen::Matrix3d::Identity() * noise_accel_bias_;
  Q.block<3, 3>(kIndexNoiseBiasGyro, kIndexNoiseBiasGyro) =
    Eigen::Matrix3d::Identity() * noise_gyro_bias_;
  Q *= dt;
  X_ = F_ * X_;
  P_ = F_ * P_ * F_.transpose() + B_ * Q * B_.transpose();
  return true;
}

bool Eskf::observe_point_cloud(const std::vector<PlaneObservation> & observations)
{
  if (!is_inited_ || observations.empty()) {
    return false;
  }
  const auto & state = imu_integration_.get_imu_nav_state();
  // iterative update: the residuals are re-linearized at the updated state
  Eigen::Matrix<double, kDimState, 1> dx_total = Eigen::Matrix<double, kDimState, 1>::Zero();
  Eigen::Matrix3d R = state.orientation;
  Eigen::Vector3d t = state.position;
  int num_observation = static_cast<int>(observations.size());
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_observation, kDimState);
  Eigen::VectorXd r = Eigen::VectorXd::Zero(num_observation);
  // information form: gain = (P^-1 + H' R^-1 H)^-1 H' R^-1, R = noise * I. the covariance
  // form needs an n x n innovation matrix, this one is 18 x 18 while H' H is O(n^2). it
  // is built from the prior once, the iterations only re-linearize (re-updating the
  // covariance would over-shrink it)
  const double inv_noise = 1.0 / observation_noise_;
  const Eigen::Matrix<double, kDimState, kDimState> identity =
    Eigen::Matrix<double, kDimState, kDimState>::Identity();
  // ponytail: dense 18x18 inverse, fine at this size; use LDLT of the
  // information matrix directly if the state ever grows
  for (int i = 0; i < num_observation; i++) {
    const auto & obs = observations[i];
    H.block<1, 3>(i, kIndexErrorPos) = obs.plane_normal.transpose();
    H.block<1, 3>(i, kIndexErrorOri) =
      -obs.plane_normal.transpose() * R * Sophus::SO3d::hat(obs.point);
  }
  Eigen::Matrix<double, kDimState, kDimState> S =
    P_.ldlt().solve(identity) + inv_noise * H.transpose() * H;
  Eigen::LDLT<Eigen::Matrix<double, kDimState, kDimState>> llt(S);
  Eigen::Matrix<double, kDimState, kDimState> P_new = llt.solve(identity);
  for (int iter = 0; iter < max_num_iterations_; iter++) {
    // residuals & jacobians at the current linearization point (R, t)
    for (int i = 0; i < num_observation; i++) {
      const auto & obs = observations[i];
      const Eigen::Vector3d & p = obs.point;
      const Eigen::Vector3d & n = obs.plane_normal;
      Eigen::Vector3d p_world = R * p + t;
      r(i) = n.dot(p_world - obs.plane_point);
      // d(n' * (R * Exp(dx_ori) * p + t + dx_pos - q)) / dx
      H.block<1, 3>(i, kIndexErrorPos) = n.transpose();
      H.block<1, 3>(i, kIndexErrorOri) = -n.transpose() * R * Sophus::SO3d::hat(p);
    }
    Eigen::Matrix<double, kDimState, 1> dx = llt.solve(H.transpose() * r) * inv_noise;
    // inject the total error into the linearization point, subtracted from the state
    // (see eliminate_error)
    t -= dx.block<3, 1>(kIndexErrorPos, 0);
    R = R * Sophus::SO3d::exp(-dx.block<3, 1>(kIndexErrorOri, 0)).matrix();
    dx_total += dx;
    if (
      dx.block<3, 1>(kIndexErrorPos, 0).norm() < converge_threshold_ &&
      dx.block<3, 1>(kIndexErrorOri, 0).norm() < converge_threshold_)
    {
      break;
    }
  }
  P_ = P_new;
  X_ = dx_total;
  eliminate_error();
  return true;
}

void Eskf::eliminate_error()
{
  auto state = imu_integration_.get_imu_nav_state();
  const auto & dx = X_;
  state.position -= dx.block<3, 1>(kIndexErrorPos, 0);
  state.linear_velocity -= dx.block<3, 1>(kIndexErrorVel, 0);
  // right perturbation
  Eigen::Matrix3d dR = Sophus::SO3d::exp(-dx.block<3, 1>(kIndexErrorOri, 0)).matrix();
  state.orientation = state.orientation * dR;
  state.accel_bias -= dx.block<3, 1>(kIndexErrorAccel, 0);
  state.gyro_bias -= dx.block<3, 1>(kIndexErrorGyro, 0);
  state.gravity -= dx.block<3, 1>(kIndexErrorGravity, 0);
  imu_integration_.reset(state);
  X_.setZero();
}

double Eskf::get_time()
{
  return imu_integration_.get_imu_nav_state().time;
}

slt_common::ImuNavState Eskf::get_imu_nav_state()
{
  return imu_integration_.get_imu_nav_state();
}

slt_common::ImuData Eskf::get_imu_data()
{
  return imu_integration_.get_imu_data();
}

}  // namespace slt_lio
