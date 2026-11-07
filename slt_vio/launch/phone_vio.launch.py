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
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slt_vio = get_package_share_directory('slt_vio')
    calibration_config = os.path.join(pkg_slt_vio, 'resource', 'phone_calibration.yaml')
    vio_config = os.path.join(pkg_slt_vio, 'config', 'phone_vio_odometry.yaml')
    rviz2_config = os.path.join(pkg_slt_vio, 'launch', 'phone_vio.rviz')
    # the calibration is the rig: extrinsics on static tf, intrinsics on the latched camera_info
    calibration_node = Node(
        name='calib_manager_node',
        package='slt_common',
        executable='calib_manager_node',
        parameters=[{'calibration_config': calibration_config}],
        output='screen',
    )
    # the base frame is the camera: the calibration's static tf is camera -> imu, so the odometry
    # tf (odom -> base) would give imu a second parent
    vio_node = Node(
        name='vio_node',
        package='slt_vio',
        executable='vio_node',
        parameters=[
            {
                'vio_config': vio_config,
                'enable_compressed': True,
                'base_frame_id': 'camera',
                'camera_frame_id': 'camera',
                'imu_frame_id': 'imu',
                'odom_frame_id': 'map',
            }
        ],
        remappings=[
            ('image', '/camera/image_raw/compressed'),
            ('imu', '/imu/data'),
        ],
        output='screen',
    )
    # one odometry and no ground truth: the phone bag is an evaluation-by-eye walk
    simple_evaluator_node = Node(
        name='simple_evaluator_node',
        package='slt_common',
        executable='simple_evaluator_node',
        parameters=[
            {
                'trajectory_path': os.path.join(
                    os.environ.get('HOME', '.'), 'localization_data', 'trajectory_vio'
                ),
                'odom_names': ['odom_vio'],
                'odom_topics': ['visual_odometry/odom'],
                'reference_odom_name': 'odom_vio',
            }
        ],
        output='screen',
    )
    # the phone rviz: a room-sized view and trail, the raw frame next to the tracked one
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        output='screen',
    )
    return LaunchDescription([calibration_node, vio_node, simple_evaluator_node, rviz2])
