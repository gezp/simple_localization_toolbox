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

#include "slt_vio/vins_mono.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "slt_common/sensor_data_utils.hpp"
#include "slt_vio/feature_utils.hpp"

namespace slt_vio
{
VinsMono::VinsMono(const YAML::Node & config)
: config_(config)
{
  freq_ = config_["freq"].as<int>(freq_);
  td_ = config_["td"].as<double>(td_);
  window_size_ = config_["window_size"].as<int>(window_size_);
  focal_length_ = config_["focal_length"].as<double>(focal_length_);
  // keyframe_parallax is in virtual-camera pixels, the comparison on the normalized image
  min_parallax_ = config_["keyframe_parallax"].as<double>(10.0) / focal_length_;
  accel_noise_ = config_["acc_noise"].as<double>(accel_noise_);
  gyro_noise_ = config_["gyr_noise"].as<double>(gyro_noise_);
  accel_bias_noise_ = config_["acc_walk"].as<double>(accel_bias_noise_);
  gyro_bias_noise_ = config_["gyr_walk"].as<double>(gyro_bias_noise_);
  // optimizer reads its own section; keys it shares with the window are repeated in it
  optimizer_ = std::make_unique<Optimizer>(config_["optimizer"]);
  // initialization reads its own section; keys it shares with the window are repeated in it
  initializer_ = std::make_unique<VisualInertialInitializer>(config_["initializer"]);
  // built here, but nothing is tracked before its intrinsics have arrived
  feature_tracker_ = std::make_unique<FeatureTracker>(config_["feature_tracker"]);
  elapsed_time_statistics_.set_enable(config_["enable_elapsed_time_statistics"].as<bool>(false));
  elapsed_time_statistics_.set_title("VinsMono");
  clear_state();
}

bool VinsMono::set_camera_intrinsic(const slt_common::CameraIntrinsicParam & param)
{
  if (is_valid_intrinsic_) {
    return true;
  }
  slt_common::CameraModel camera_model;
  if (!camera_model.set_intrinsic_param(param)) {
    return false;
  }
  feature_tracker_->set_camera_model(camera_model);
  is_valid_intrinsic_ = true;
  return true;
}

void VinsMono::set_extrinsic(const Eigen::Matrix4d & T_imu_camera)
{
  // the extrinsic is fixed by the rig: both users take it now, not per solve
  initializer_->set_extrinsic(T_imu_camera);
  optimizer_->set_extrinsic(T_imu_camera);
  is_valid_extrinsic_ = true;
}

void VinsMono::add_imu_data(const slt_common::ImuData & imu_data)
{
  // samples are held here and nowhere else; every frame reads the ones up to its stamp
  imu_buffer_.push_back(imu_data);
  // ponytail: a stalled consumer loses the oldest sample and any interval spanning the hole is
  // integrated short. drop the backlog and reboot the window instead if that ever happens
  if (imu_buffer_.size() > kMaxImuBufferSize) {
    imu_buffer_.pop_front();
  }
}

void VinsMono::add_image_data(const slt_common::ImageData & image_data)
{
  image_buffer_.push_back(image_data);
  // the oldest falls off; the gap makes the stream gate treat what is left as a new stream
  if (image_buffer_.size() > kMaxImageBufferSize) {
    image_buffer_.pop_front();
  }
}

bool VinsMono::update()
{
  // nothing is tracked before intrinsics and extrinsic are in
  if (!is_valid_intrinsic_ || !is_valid_extrinsic_) {
    return false;
  }
  // the window is the initialization's until it settles: bootstrap frames never reach the
  // estimator, and a reboot returns here
  if (!is_inited_) {
    if (!try_init()) {
      return false;
    }
  }
  bool has_new_frame = false;
  elapsed_time_statistics_.tic("update");
  // the image waits for the imu sample crossing its stamp, else the interval integrates short and
  // the scale drifts; the stamp is the frame's, the image one shifted by the time offset. several
  // frames in one call collapse to their newest pose.
  // ponytail: a stalled imu stream holds the images back, drop the backlog if that ever matters
  while (!image_buffer_.empty() && !imu_buffer_.empty() &&
    imu_buffer_.back().time > image_buffer_.front().time + td_)
  {
    const slt_common::ImageData image_data = image_buffer_.front();
    image_buffer_.pop_front();
    // a solve that reboots the window leaves the remaining frames to the bootstrap again
    if (!process_image(image_data)) {
      break;
    }
    has_new_frame = true;
  }
  elapsed_time_statistics_.toc("update");
  elapsed_time_statistics_.print_all_info("update", 100);
  return has_new_frame && is_inited_;
}

bool VinsMono::try_init()
{
  bool inited = false;
  while (!image_buffer_.empty() && !imu_buffer_.empty() &&
    imu_buffer_.back().time > image_buffer_.front().time + td_)
  {
    const slt_common::ImageData image_data = image_buffer_.front();
    image_buffer_.pop_front();
    if (!track_image(image_data)) {
      continue;
    }
    // the initialization rolls its own window until it settles; the frame is handed over and
    // processed right away, so a window settling before it leaves it to the estimator
    const FeatureFrame feature_frame = build_feature_frame(image_data);
    // the interval is born at zero bias: the alignment solves it
    const auto pre_integration = build_imu_pre_integration(
      feature_frame.time, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    // the first frame opens the window with no interval; the initialization drops it
    initializer_->add_frame(feature_frame, pre_integration, check_new_key_frame(feature_frame));
    last_frame_id_ = feature_frame.frame_id;
    if (initializer_->try_init()) {
      inited = true;
      break;
    }
  }
  message_ += initializer_->error_message();
  if (!inited) {
    return false;
  }
  // the settled window is the estimator's from here, registered and solved
  init_optimizer_graph();
  is_inited_ = true;
  // the initialization gathers its own window again
  initializer_->reset();
  return true;
}

FeatureFrame VinsMono::build_feature_frame(const slt_common::ImageData & image_data) const
{
  const TrackedFeatures & tracked = feature_tracker_->get_features();
  FeatureFrame feature_frame;
  feature_frame.time = image_data.time + td_;
  // id the graph keys the frame by: the image time stamp in nanoseconds
  feature_frame.frame_id = static_cast<uint64_t>(image_data.time * 1e9);
  for (size_t i = 0; i < tracked.ids.size(); i++) {
    FeatureObservation observation;
    observation.uv << tracked.pts[i].x, tracked.pts[i].y;
    observation.xy << tracked.un_pts[i].x, tracked.un_pts[i].y;
    observation.v_xy << tracked.velocity[i].x, tracked.velocity[i].y;
    feature_frame.features[tracked.ids[i]] = observation;
  }
  return feature_frame;
}

bool VinsMono::process_image(const slt_common::ImageData & image_data)
{
  if (!track_image(image_data)) {
    return false;
  }
  elapsed_time_statistics_.tic("process_image");
  slt_common::ImuNavState state = optimizer_->get_imu_nav_state();
  const FeatureFrame feature_frame = build_feature_frame(image_data);
  const auto pre_integration = build_imu_pre_integration(
    feature_frame.time, state.accel_bias, state.gyro_bias);
  slt_common::ImuNavState propagated_state;
  if (pre_integration->apply(state, propagated_state)) {
    state = propagated_state;
  }
  state.time = feature_frame.time;

  // verdict on this frame, measured against the key frames before it lands; joins them if one
  optimizer_->add_frame(feature_frame.frame_id, state, check_new_key_frame(feature_frame));
  optimizer_->add_imu_edge(last_frame_id_, feature_frame.frame_id, pre_integration);
  optimizer_->add_feature(feature_frame);

  last_frame_id_ = feature_frame.frame_id;
  // the solve: triangulates what has no depth yet, keeps the depths itself
  optimizer_->optimize();
  message_ += optimizer_->error_message();
  // a solve that reboots the window leaves the remaining frames to the bootstrap again
  if (failure_detection()) {
    reboot_count_++;
    clear_state();
    elapsed_time_statistics_.toc("process_image");
    return false;
  }
  // the slide, spent on the verdict the previous frame landed with
  // optimizer_->marginalize();
  while (optimizer_->frame_count() > static_cast<size_t>(window_size_)) {
    optimizer_->marginalize();
  }
  // state the window ends on, which the next frame is checked against
  last_nav_state_ = optimizer_->get_imu_nav_state();
  elapsed_time_statistics_.toc("process_image");
  return true;
}

bool VinsMono::track_image(const slt_common::ImageData & image_data)
{
  if (!accept_image(image_data.time)) {
    return false;
  }
  const bool output_frame = is_output_frame(image_data.time);
  feature_tracker_->read_image(image_data.image, image_data.time, output_frame);
  // the display draws the tracking of the image that landed last
  tracked_image_ = image_data.image;
  return output_frame;
}

slt_imu_odometry::ImuPreIntegrationPtr VinsMono::build_imu_pre_integration(
  double time, const Eigen::Vector3d & ba, const Eigen::Vector3d & bg)
{
  // no opener: the stream's first frame; the samples up to it only set the opener at the frame time
  if (!has_last_imu_) {
    while (!imu_buffer_.empty() && imu_buffer_.front().time <= time) {
      last_imu_ = imu_buffer_.front();
      imu_buffer_.pop_front();
    }
    last_imu_ = slt_common::interpolate_imu(last_imu_, imu_buffer_.front(), time);
    has_last_imu_ = true;
    return nullptr;
  }
  // the interval opens where the previous one ended, takes every sample up to the frame time and
  // ends there: the crossing sample is split, its second half opens the next interval
  std::vector<slt_common::ImuData> samples;
  samples.push_back(last_imu_);
  while (!imu_buffer_.empty() && imu_buffer_.front().time <= time) {
    const slt_common::ImuData imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    // a sample not after the opener belongs to an already-closed interval: drop it
    if (imu.time <= last_imu_.time) {
      continue;
    }
    samples.push_back(imu);
  }
  // the sample the frame time falls on is split there; its second half opens the next interval
  last_imu_ = slt_common::interpolate_imu(samples.back(), imu_buffer_.front(), time);
  samples.push_back(last_imu_);

  // built at the bias of the state the interval is applied to
  const slt_imu_odometry::ImuPreIntegrationPtr pre_integration =
    std::make_shared<slt_imu_odometry::ImuPreIntegration>(
    accel_noise_, gyro_noise_, accel_bias_noise_, gyro_bias_noise_);
  pre_integration->set_bias(ba, bg);
  // the interval keeps the samples it integrated; a non-key frame hands them over
  for (const slt_common::ImuData & imu : samples) {
    pre_integration->integrate(imu);
  }

  return pre_integration;
}

bool VinsMono::accept_image(double time)
{
  if (is_first_image_) {
    is_first_image_ = false;
    first_image_time_ = time;
    last_image_time_ = time;
    return false;
  }
  if (time - last_image_time_ > kImageDiscontinueGap || time < last_image_time_) {
    is_first_image_ = true;
    last_image_time_ = 0;
    pub_count_ = 1;
    return false;
  }
  last_image_time_ = time;
  return true;
}

bool VinsMono::is_output_frame(double time)
{
  // a non positive freq keeps the raw image rate
  if (freq_ <= 0) {
    return true;
  }
  const double elapsed = time - first_image_time_;
  const double rate = elapsed > 0 ? 1.0 * pub_count_ / elapsed : 0;
  if (std::round(rate) > freq_) {
    return false;
  }
  // reset the frequency control once the target rate is reached
  if (std::abs(rate - freq_) < 0.01 * freq_) {
    first_image_time_ = time;
    pub_count_ = 0;
  }
  pub_count_++;
  return true;
}

slt_common::ImuNavState VinsMono::get_imu_nav_state() const
{
  // the graph holds the window between two frames; propagation only moves its copy
  return optimizer_->get_imu_nav_state();
}

cv::Mat VinsMono::get_feature_image() const
{
  cv::Mat show_img;
  if (tracked_image_.empty()) {
    return show_img;
  }
  if (tracked_image_.channels() == 3) {
    show_img = tracked_image_.clone();
  } else {
    cv::cvtColor(tracked_image_, show_img, cv::COLOR_GRAY2BGR);
  }
  const TrackedFeatures & features = feature_tracker_->get_features();
  for (size_t i = 0; i < features.pts.size(); i++) {
    // color runs from red to blue as the feature is tracked longer
    const double len = std::min(1.0, 1.0 * features.track_cnt[i] / kTrackCountForViz);
    cv::circle(show_img, features.pts[i], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
  }
  return show_img;
}

std::string VinsMono::error_message()
{
  std::string message = message_;
  message_.clear();
  return message;
}

void VinsMono::clear_state()
{
  last_imu_ = slt_common::ImuData{};
  has_last_imu_ = false;
  imu_buffer_.clear();
  last_nav_state_ = slt_common::ImuNavState{};
  last_frame_id_ = 0;
  // no key frame to judge the next frame against until the initialization hands one over
  key_frames_.clear();
  is_inited_ = false;
  // the initialization gathers its own window again
  initializer_->reset();
  optimizer_->clear_state();
}

bool VinsMono::is_key_frame(const FeatureFrame & feature_frame) const
{
  // nothing to judge against yet: a frame before the history holds one is a key frame
  if (key_frames_.size() < 2) {
    return true;
  }
  // what the window's key frames still hold of it decides, not what the tracker saw
  int tracked = 0;
  for (const auto & id_pts : feature_frame.features) {
    for (const auto & key_frame : key_frames_) {
      if (key_frame.features.count(id_pts.first)) {
        tracked++;
        break;
      }
    }
  }
  // the tracker almost lost the frame
  if (tracked < kMinTrackedFeatures) {
    return true;
  }
  // far enough from the newest key frame, or nothing shared left to measure
  if (has_enough_parallax(feature_frame.features, key_frames_.back().features, min_parallax_)) {
    return true;
  }
  return false;
}

bool VinsMono::check_new_key_frame(const FeatureFrame & feature_frame)
{
  if (!is_key_frame(feature_frame)) {
    return false;
  }
  // the key frame joins the window's; key frames keep one fewer than the window does frames
  key_frames_.push_back(feature_frame);
  while (key_frames_.size() >= static_cast<size_t>(window_size_)) {
    key_frames_.pop_front();
  }
  return true;
}

void VinsMono::init_optimizer_graph()
{
  // the settled window enters the graph: its frames with the alignment's states, the intervals
  // ending at them; solved once, nothing slides
  for (const auto & frame : initializer_->frames()) {
    const uint64_t frame_id = frame.frame_id;
    // the initialization hands its whole window over, every frame a key frame
    const FeatureFrame feature_frame{frame_id, frame.time, frame.features};
    optimizer_->add_frame(frame_id, frame.state, true);
    optimizer_->add_feature(feature_frame);
  }
  for (const auto & edge : initializer_->imu_edges()) {
    optimizer_->add_imu_edge(edge.from, edge.to, edge.pre_integration);
  }

  // solved once, so the estimator starts from a solved window; nothing slides, the first frame
  // the estimator takes makes room for itself
  optimizer_->optimize();
  message_ += optimizer_->error_message();
  // state the settled window is on, which the first frame that follows is checked against
  last_nav_state_ = optimizer_->get_imu_nav_state();
}

bool VinsMono::failure_detection()
{
  const slt_common::ImuNavState newest = optimizer_->get_imu_nav_state();
  if (newest.accel_bias.norm() > kMaxAccBias) {
    message_ = "big imu acc bias estimation";
    return true;
  }
  if (newest.gyro_bias.norm() > kMaxGyrBias) {
    message_ = "big imu gyr bias estimation";
    return true;
  }
  const Eigen::Vector3d tmp_p = newest.position;
  const Eigen::Vector3d last_p = last_nav_state_.position;
  if ((tmp_p - last_p).norm() > kMaxTranslation) {
    message_ = "big translation";
    return true;
  }
  if (std::abs(tmp_p.z() - last_p.z()) > kMaxZTranslation) {
    message_ = "big z translation";
    return true;
  }
  return false;
}

}  // namespace slt_vio
