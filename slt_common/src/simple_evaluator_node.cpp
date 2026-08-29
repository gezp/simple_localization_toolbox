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

#include "slt_common/simple_evaluator_node.hpp"

#include <filesystem>

namespace slt_common
{
SimpleEvaluatorNode::SimpleEvaluatorNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // parameters
  std::string data_path;
  std::string reference_odom_name;
  node->declare_parameter("trajectory_path", trajectory_path_);
  node->declare_parameter("odom_names", odom_names_);
  node->declare_parameter("odom_topics", odom_topics_);
  node->declare_parameter("reference_odom_name", reference_odom_name);
  node->get_parameter("trajectory_path", trajectory_path_);
  node->get_parameter("odom_names", odom_names_);
  node->get_parameter("odom_topics", odom_topics_);
  node->get_parameter("reference_odom_name", reference_odom_name);
  // trajectory_path
  if (trajectory_path_ == "") {
    RCLCPP_FATAL(node->get_logger(), "trajectory_path is empty.");
    return;
  }
  if (!std::filesystem::is_directory(std::filesystem::path(trajectory_path_).parent_path())) {
    RCLCPP_FATAL(node->get_logger(), "trajectory_path is invalid: %s", trajectory_path_.c_str());
    return;
  }
  RCLCPP_INFO(node->get_logger(), "trajectory storage path: %s", trajectory_path_.c_str());
  // odom_names & odom_topics
  if (odom_topics_.size() != odom_names_.size()) {
    RCLCPP_FATAL(node->get_logger(), "the size of odom_topics must equal to odom_names");
    return;
  }
  if (odom_names_.size() == 0) {
    RCLCPP_FATAL(node->get_logger(), "at least 1 odom topic");
    return;
  }
  if (!check_unique_element(odom_names_) || !check_unique_element(odom_topics_)) {
    RCLCPP_FATAL(node->get_logger(), "the elements in odom_names and odom_topics must be unique");
    return;
  }
  // create subscriber
  for (size_t i = 0; i < odom_names_.size(); i++) {
    auto odom_sub = std::make_shared<OdometrySubscriber>(node, odom_topics_[i], 10000);
    odom_subs_.push_back(odom_sub);
    odom_data_buffers_.push_back(OdomDataBuffer());
    map_odom_tf_published_.push_back(false);
    RCLCPP_INFO(
      node->get_logger(), "record odom[%s] on topic:%s", odom_names_[i].c_str(),
      odom_topics_[i].c_str());
  }
  // alignment tf
  static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  // reference_odom
  for (size_t i = 0; i < odom_names_.size(); i++) {
    if (odom_names_[i] == reference_odom_name) {
      reference_odom_index_ = i;
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "use odom[%s] as timestamp reference.",
    odom_names_[reference_odom_index_].c_str());
  // srv
  save_odometry_srv_ = node->create_service<slt_interface::srv::SaveOdometry>(
    "save_odometry",
    [this](
      const slt_interface::srv::SaveOdometry::Request::SharedPtr /*request*/,
      slt_interface::srv::SaveOdometry::Response::SharedPtr response) {
      save_odometry_flag_ = true;
      response->succeed = true;
    });
  //
  run_thread_ = std::make_unique<std::thread>(
    [this]() {
      while (!exit_) {
        run();
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
      }
    });
}

SimpleEvaluatorNode::~SimpleEvaluatorNode()
{
  exit_ = true;
  if (run_thread_) {
    run_thread_->join();
  }
}

bool SimpleEvaluatorNode::run()
{
  // update odom data
  for (size_t i = 0; i < odom_topics_.size(); i++) {
    std::deque<OdomData> odom_buffer;
    odom_subs_[i]->parse_data(odom_buffer);
    for (auto & data : odom_buffer) {
      odom_data_buffers_[i].add_data(data);
    }
    if (!map_odom_tf_published_[i] && !odom_buffer.empty() && i != reference_odom_index_) {
      map_odom_tf_published_[i] = publish_map_odom_tf(odom_names_[i], odom_buffer.front());
    }
  }
  if (save_odometry_flag_) {
    save_trajectory();
    save_odometry_flag_ = false;
  }
  return true;
}

bool SimpleEvaluatorNode::publish_map_odom_tf(const std::string & odom_name, const OdomData & odom)
{
  // align first odom pose to reference at same timestamp
  OdomData ref_odom;
  if (!odom_data_buffers_[reference_odom_index_].get_interpolated_data(odom.time, ref_odom)) {
    // wait reference data
    return false;
  }
  Eigen::Matrix4d T_map_odom = ref_odom.pose * odom.pose.inverse();
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = map_frame_id_;
  msg.child_frame_id = odom_name;
  msg.transform = to_transform_msg(T_map_odom);
  static_tf_pub_->sendTransform(msg);
  Eigen::Vector3d p = T_map_odom.block<3, 1>(0, 3);
  RCLCPP_INFO(
    node_->get_logger(), "publish static tf %s -> %s at position: (%lf, %lf, %lf)",
    msg.header.frame_id.c_str(), msg.child_frame_id.c_str(), p.x(), p.y(), p.z());
  return true;
}

void SimpleEvaluatorNode::save_pose(std::ofstream & ofs, const OdomData & odom)
{
  // for evo
  // timestamp tx ty tz qx qy qz qw
  Eigen::Vector3d t = odom.pose.block<3, 1>(0, 3);
  Eigen::Quaterniond q = Eigen::Quaterniond(odom.pose.block<3, 3>(0, 0));
  ofs << odom.time << " " << t.x() << " " << t.y() << " " << t.z() << " ";
  ofs << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
}

bool SimpleEvaluatorNode::save_trajectory()
{
  if (std::filesystem::is_directory(trajectory_path_)) {
    std::filesystem::remove_all(trajectory_path_);
  }
  if (!std::filesystem::create_directory(trajectory_path_)) {
    RCLCPP_INFO(node_->get_logger(), "failed to create trajectory directory.");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "start to save trajectory");
  // get intersection of all odom buffers [start_time, end_time]
  double start_time = 0;
  double end_time = std::numeric_limits<double>::max();
  for (size_t i = 0; i < odom_names_.size(); i++) {
    if (odom_data_buffers_[i].size() == 0) {
      RCLCPP_INFO(node_->get_logger(), "no data in odom [%s], skip", odom_names_[i].c_str());
      continue;
    }
    double s = odom_data_buffers_[i].get_start_time();
    double e = odom_data_buffers_[i].get_end_time();
    RCLCPP_INFO(
      node_->get_logger(), "odom [%s]: timestamps[%lf, %lf]", odom_names_[i].c_str(), s, e);
    start_time = std::max(start_time, s);
    end_time = std::min(end_time, e);
  }
  RCLCPP_INFO(node_->get_logger(), "all odometry timestamps[%lf, %lf]", start_time, end_time);
  if (start_time > end_time) {
    RCLCPP_ERROR(
      node_->get_logger(), "failed to save trajectory due to no common timestamps for all odoms");
    return false;
  }
  // timestamp from reference_odom buffer
  if (odom_data_buffers_[reference_odom_index_].size() == 0) {
    RCLCPP_ERROR(
      node_->get_logger(), "failed to save trajectory due to no data in reference_odom [%s]",
      odom_names_[reference_odom_index_].c_str());
    return false;
  }
  auto timestamp_buffer = odom_data_buffers_[reference_odom_index_].get_vector();
  size_t start_index = timestamp_buffer.size() - 1;
  size_t end_index = 0;
  for (size_t i = 0; i < timestamp_buffer.size(); i++) {
    if (timestamp_buffer[i].time >= start_time && timestamp_buffer[i].time <= end_time) {
      // valid timestamp
      start_index = std::min(start_index, i);
      end_index = std::max(end_index, i);
    }
  }
  if (start_index > end_index) {
    RCLCPP_ERROR(node_->get_logger(), "failed to save trajectory due to invalid timestamp");
    return false;
  }
  size_t valid_cnt = end_index - start_index + 1;
  size_t invalid_cnt = timestamp_buffer.size() - valid_cnt;
  RCLCPP_INFO(
    node_->get_logger(),
    "reference odometry [%s]: remove %lu, total valid timestamp index [%lu, %lu] (%lu)",
    odom_names_[reference_odom_index_].c_str(), invalid_cnt, start_index, end_index, valid_cnt);
  // save odoms
  for (size_t i = 0; i < odom_names_.size(); i++) {
    if (odom_data_buffers_[i].size() == 0) {
      continue;
    }
    auto name = odom_names_[i];
    std::string path = trajectory_path_ + +"/" + name + ".txt";
    std::ofstream trajectory_ofs;
    trajectory_ofs.open(path, std::ios::app);
    if (!trajectory_ofs) {
      RCLCPP_FATAL(
        node_->get_logger(), "odom [%s] failed to open path %s", name.c_str(), path.c_str());
      return false;
    }
    trajectory_ofs.setf(std::ios::fixed, std::ios::floatfield);
    trajectory_ofs.precision(4);
    trajectory_ofs << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    for (size_t j = start_index; j <= end_index; j++) {
      OdomData odom;
      if (!odom_data_buffers_[i].get_interpolated_data(timestamp_buffer[j].time, odom)) {
        RCLCPP_FATAL(
          node_->get_logger(), "odom [%s] failed to get interpolated data at %lf.", name.c_str(),
          timestamp_buffer[j].time);
        return false;
      }
      save_pose(trajectory_ofs, odom);
    }
    RCLCPP_INFO(node_->get_logger(), "odom [%s] successed to save trajectory.", name.c_str());
  }
  RCLCPP_INFO(node_->get_logger(), "finish to save all trajectory.");
  return true;
}

bool SimpleEvaluatorNode::check_unique_element(const std::vector<std::string> & v)
{
  std::set<std::string> s(v.begin(), v.end());
  return s.size() == v.size();
}

}  // namespace slt_common
