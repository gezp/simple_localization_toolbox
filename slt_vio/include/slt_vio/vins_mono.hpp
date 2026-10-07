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

#pragma once

#include <Eigen/Dense>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "slt_common/camera_model/camera_intrinsic_param.hpp"
#include "slt_common/math_utils.hpp"
#include "slt_common/sensor_data/image_data.hpp"
#include "slt_common/sensor_data/imu_data.hpp"
#include "slt_common/sensor_data/imu_nav_state.hpp"
#include "slt_common/tic_toc.hpp"
#include "slt_imu_odometry/imu_pre_integration.hpp"
#include "slt_vio/feature.hpp"
#include "slt_vio/feature_tracker.hpp"
#include "slt_vio/initializer/visual_inertial_initializer.hpp"
#include "slt_vio/optimizer/optimizer.hpp"
#include "yaml-cpp/yaml.h"

namespace slt_vio
{
// VINS-Mono style monocular visual inertial odometry: owns the feature tracker and the sliding
// window estimator. an image reaches the tracker only once the imu sample crossing its stamp
// has arrived. reference: VINS-Mono (https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
class VinsMono
{
public:
  // the `vins_mono` config section; tracker, initializer and optimizer read their own subsections
  explicit VinsMono(const YAML::Node & config);

  // camera intrinsics arrive on a topic after start; nothing is tracked before them
  bool set_camera_intrinsic(const slt_common::CameraIntrinsicParam & param);
  // T_imu_camera: camera frame -> imu frame, fixed by the rig, must be in before the first frame
  void set_extrinsic(const Eigen::Matrix4d & T_imu_camera);
  void add_imu_data(const slt_common::ImuData & imu_data);
  void add_image_data(const slt_common::ImageData & image_data);
  // true when a frame left a new window state behind
  bool update();

  // the window's world frame; meaningful only once update() has returned true
  slt_common::ImuNavState get_imu_nav_state() const;
  // empty before the first frame
  cv::Mat get_feature_image() const;
  // carries the state's angular velocity; the frame's interval ends on it, the next opens from it
  slt_common::ImuData get_imu_data() const {return last_imu_;}
  // drains and clears the error log the module raised since the last call
  std::string error_message();
  // times the window was rebooted after a failure
  int get_reboot_count() const {return reboot_count_;}

private:
  // false when the image never becomes a frame: vins decimates to a fixed rate and drops it
  bool track_image(const slt_common::ImageData & image_data);
  // the id the graph keys the frame by, its time stamp, and what it saw
  FeatureFrame build_feature_frame(const slt_common::ImageData & image_data) const;
  // false while the bootstrap is still short of frames
  bool try_init();
  // false when the image yields no frame or the solve reboots the window
  bool process_image(const slt_common::ImageData & image_data);
  // the interval ending at the frame: samples up to the image time, the one crossing it split
  // there. null for the stream's first frame, which only sets the opener
  slt_imu_odometry::ImuPreIntegrationPtr build_imu_pre_integration(
    double time, const Eigen::Vector3d & ba, const Eigen::Vector3d & bg);
  // false on the stream's first frame and on one after a gap: both only re-seed the clock
  bool accept_image(double time);
  // vins keeps the estimator at a fixed rate, decimating the image stream to it
  bool is_output_frame(double time);

  bool failure_detection();
  void clear_state();

  // true when it shares too little with the window's key frames or moved far from the newest
  bool is_key_frame(const FeatureFrame & feature_frame) const;
  // joins the window's key frames, which keep one fewer than the window does frames
  bool check_new_key_frame(const FeatureFrame & feature_frame);
  // the settled window enters the graph — frames, intervals, features, depths — and is solved
  // once; nothing slides, the window is one frame over the steady size
  void init_optimizer_graph();

private:
  // the image stream is treated as discontinuous after such a gap
  static constexpr double kImageDiscontinueGap = 1.0;
  // the thresholds of the failure detection
  static constexpr double kMaxAccBias = 2.5;
  static constexpr double kMaxGyrBias = 1.0;
  static constexpr double kMaxTranslation = 5.0;
  static constexpr double kMaxZTranslation = 1.0;
  // track count at which a feature is drawn fully saturated
  static constexpr int kTrackCountForViz = 10;
  // below this many features carried over from the previous frame, the frame is a key frame
  static constexpr int kMinTrackedFeatures = 20;
  // buffers cap at ~10 s of stream at imu 200 Hz and images 10 Hz, dropping the oldest
  static constexpr size_t kMaxImuBufferSize = 2000;
  static constexpr size_t kMaxImageBufferSize = 100;
  int window_size_{10};
  // virtual camera focal length, which the config's key frame parallax is divided by
  double focal_length_{460.0};
  // key-frame parallax threshold, normalized image; the config gives it in virtual-camera pixels
  double min_parallax_{0.0};
  // preintegration noise: accel/gyro noise densities, then the two bias random walks
  double accel_noise_{0.1};
  double gyro_noise_{0.01};
  double accel_bias_noise_{0.001};
  double gyro_bias_noise_{0.0001};

  YAML::Node config_;
  int freq_{10};
  // the offset of the image clock against the imu one
  double td_{0.0};
  // set by the setters once their topic has delivered
  bool is_valid_intrinsic_{false};
  bool is_valid_extrinsic_{false};

  std::unique_ptr<FeatureTracker> feature_tracker_;
  // owns the window and the frames handed to it; an interval's recursion runs on the
  // preintegration ending at the frame, only its state lands in the window
  std::unique_ptr<Optimizer> optimizer_;
  // samples that have arrived and no frame has read: every frame builds its ending interval from
  // them, the stream's first only to set the opener
  std::deque<slt_common::ImuData> imu_buffer_;
  std::deque<slt_common::ImageData> image_buffer_;

  bool is_first_image_{true};
  double first_image_time_{0};
  double last_image_time_{0};
  int pub_count_{1};
  cv::Mat tracked_image_;

  // set by the hand over, cleared by a reboot
  bool is_inited_{false};

  // owns the bootstrap's frame window and rolls it until it settles
  std::unique_ptr<VisualInertialInitializer> initializer_;

  // the last output frame's interval ends on it; the following interval opens from it
  slt_common::ImuData last_imu_;
  // set by the stream's first frame; build_imu_pre_integration is the only reader
  bool has_last_imu_{false};
  // state at the last output frame; failure_detection measures the next frame against it
  slt_common::ImuNavState last_nav_state_;
  // frame that landed last in the window: the next frame's interval opens at it
  uint64_t last_frame_id_{0};
  // key frames in landing order; the verdict measures a new frame against the newest
  std::deque<FeatureFrame> key_frames_;

  std::string message_;
  int reboot_count_{0};
  slt_common::AdvancedTicToc elapsed_time_statistics_;
};
}  // namespace slt_vio
