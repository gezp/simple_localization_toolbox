# Copyright 2026 Zhenpeng Ge (https://github.com/zhenpengge).
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
    pkg_slt_lio = get_package_share_directory('slt_lio')
    rviz2_config = os.path.join(pkg_slt_lio, 'launch', 'lio_odometry.rviz')
    data_dir = os.path.join(os.environ['HOME'], 'localization_data')
    lio_config = os.path.join(pkg_slt_lio, 'config', 'lio_odometry.yaml')
    bag_path = os.path.join(data_dir, 'kitti_lidar_only_2011_10_03_drive_0027_synced')
    rosbag_node = ExecuteProcess(
        name='rosbag',
        cmd=['ros2 bag play', bag_path, '-d 3', '--read-ahead-queue-size 1000'],
        shell=True,
        output='screen',
    )
    kitti_preprocess_node = Node(
        name='kitti_preprocess_node',
        package='slt_common',
        executable='kitti_preprocess_node',
        output='screen',
    )
    lio_node = Node(
        name='lio_node',
        package='slt_lio',
        executable='lio_node',
        parameters=[
            {
                'lio_config': lio_config,
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'imu_frame_id': 'imu_link',
                'lidar_frame_id': 'velo_link',
                'odom_frame_id': 'odom_lidar',
            }
        ],
        remappings=[('imu', '/kitti/oxts/imu/extract')],
        output='screen',
    )
    simple_evaluator_node = Node(
        name='simple_evaluator_node',
        package='slt_common',
        executable='simple_evaluator_node',
        parameters=[
            {
                'trajectory_path': data_dir + '/trajectory',
                'odom_names': ['ground_truth', 'odom_lidar', 'odom_imu'],
                'odom_topics': [
                    'synced_gnss/pose',
                    'lidar_odometry/odom',
                    'lidar_odometry/imu_odom',
                ],
                'reference_odom_name': 'ground_truth',
            }
        ],
        output='screen',
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        output='screen',
    )
    ld = LaunchDescription()
    ld.add_action(rosbag_node)
    ld.add_action(kitti_preprocess_node)
    ld.add_action(lio_node)
    ld.add_action(simple_evaluator_node)
    ld.add_action(rviz2)
    return ld
