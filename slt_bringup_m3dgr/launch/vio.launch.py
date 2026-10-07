# Copyright 2026 Zhenpeng Ge (https://github.com/gezp).
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slt_bringup_m3dgr = get_package_share_directory('slt_bringup_m3dgr')
    calibration_config = os.path.join(
        pkg_slt_bringup_m3dgr, 'config', 'calibration.yaml'
    )
    rviz2_config = os.path.join(pkg_slt_bringup_m3dgr, 'launch', 'vio.rviz')
    vio_config = os.path.join(
        pkg_slt_bringup_m3dgr, 'config', 'vio_odometry.yaml'
    )
    data_dir = os.path.join(
        os.environ.get('HOME', '.'), 'localization_data', 'dataset', 'M3DGR', 'Standard'
    )
    bag_path = LaunchConfiguration('bag_path')
    gt_path = LaunchConfiguration('gt_path')
    rate = LaunchConfiguration('rate')
    rosbag_node = ExecuteProcess(
        name='rosbag',
        cmd=[
            'ros2 bag play',
            bag_path,
            '-r',
            rate,
            '-d',
            '3',
            '--read-ahead-queue-size',
            '1000',
        ],
        shell=True,
        output='screen',
    )
    replay_node = Node(
        package='slt_bringup_m3dgr',
        executable='replay_node',
        name='replay_node',
        output='screen',
        parameters=[
            {
                'calibration_config': calibration_config,
                'gt_path': gt_path,
                'rate': rate,
                'publish_tf': True,
                'rebuild_gt_rotation': True,
            }
        ],
    )
    vio_node = Node(
        name='vio_node',
        package='slt_vio',
        executable='vio_node',
        parameters=[
            {
                'vio_config': vio_config,
                'enable_compressed': True,
                'base_frame_id': 'base_link',
                'camera_frame_id': 'camera',
                'imu_frame_id': 'camera_imu',
                'odom_frame_id': 'odom_vio',
            }
        ],
        remappings=[
            ('image', '/camera/color/image_raw/compressed'),
            ('camera_info', '/camera/color/camera_info'),
            ('imu', '/camera/imu'),
        ],
        output='screen',
    )
    simple_evaluator_node = Node(
        name='simple_evaluator_node',
        package='slt_common',
        executable='simple_evaluator_node',
        parameters=[
            {
                'trajectory_path': os.path.join(
                    os.environ.get('HOME', '.'), 'localization_data', 'trajectory_vio'
                ),
                'odom_names': ['ground_truth', 'odom_vio'],
                'odom_topics': ['m3dgr/ground_truth/odom', 'visual_odometry/odom'],
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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'bag_path',
                default_value=os.path.join(data_dir, 'Outdoor01', 'rosbag2'),
                description='ROS2 bag dir converted by rosbag1_to_rosbag2.py',
            ),
            DeclareLaunchArgument(
                'gt_path',
                default_value=os.path.join(data_dir, 'Outdoor01', 'Outdoor01.txt'),
            ),
            DeclareLaunchArgument('rate', default_value='1.0'),
            rosbag_node,
            replay_node,
            vio_node,
            simple_evaluator_node,
            rviz2,
        ]
    )
