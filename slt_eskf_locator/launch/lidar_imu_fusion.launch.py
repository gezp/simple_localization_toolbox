# Copyright 2023 Gezp (https://github.com/gezp).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slt_lidar_locator = get_package_share_directory("slt_lidar_locator")
    pkg_slt_eskf_locator = get_package_share_directory("slt_eskf_locator")
    rviz2_config = os.path.join(
        pkg_slt_eskf_locator, "launch", "lidar_imu_fusion.rviz"
    )
    data_dir = os.path.join(os.environ["HOME"], "localization_data")
    bag_path = os.path.join(data_dir, "kitti_lidar_only_2011_10_03_drive_0027_synced")
    fusion_config = os.path.join(
        pkg_slt_eskf_locator, "config", "lidar_imu_fusion.yaml"
    )
    slt_lidar_locator_config = os.path.join(
        pkg_slt_lidar_locator, "config", "slt_lidar_locator.yaml"
    )
    #
    rosbag_node = ExecuteProcess(
        name="rosbag",
        cmd=["ros2 bag play", bag_path, "-d 3", "--read-ahead-queue-size 1000"],
        shell=True,
        output="screen",
    )
    kitti_preprocess_node = Node(
        name="kitti_preprocess_node",
        package="slt_common",
        executable="kitti_preprocess_node",
        output="screen",
    )
    slt_lidar_locator_node = Node(
        name="slt_lidar_locator_node",
        package="slt_lidar_locator",
        executable="slt_lidar_locator_node",
        parameters=[
            {
                "slt_lidar_locator_config": slt_lidar_locator_config,
                "data_path": data_dir,
                "base_frame_id": "base_link",
                "lidar_frame_id": "velo_link",
            }
        ],
        output="screen",
    )
    fusion_node = Node(
        name="lidar_imu_fusion_node",
        package="slt_eskf_locator",
        executable="lidar_imu_fusion_node",
        parameters=[
            {
                "config_file": fusion_config,
                "base_frame_id": "base_link",
                "imu_frame_id": "imu_link",
            }
        ],
        output="screen",
    )
    simple_evaluator_node = Node(
        name="simple_evaluator_node",
        package="slt_common",
        executable="simple_evaluator_node",
        parameters=[
            {
                "trajectory_path": data_dir + "/trajectory",
                "odom_names": ["ground_truth", "lidar_pose", "fused_pose"],
                "odom_topics": [
                    "synced_gnss/pose",
                    "localization/lidar/pose",
                    "localization/fused/pose",
                ],
                "reference_odom_name": "lidar_pose",
            }
        ],
        output="screen",
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz2_config],
        output="screen",
    )
    ld = LaunchDescription()
    ld.add_action(rosbag_node)
    ld.add_action(kitti_preprocess_node)
    ld.add_action(slt_lidar_locator_node)
    ld.add_action(fusion_node)
    ld.add_action(simple_evaluator_node)
    ld.add_action(rviz2)
    return ld
