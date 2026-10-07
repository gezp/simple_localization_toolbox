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
    vio_config = os.path.join(pkg_slt_vio, 'config', 'vio_odometry.yaml')
    rviz2_config = os.path.join(pkg_slt_vio, 'launch', 'vio.rviz')
    # the module only: the image, the camera info and the imu come off the topics the node is
    # subscribed to, and the rig that feeds them is the caller's driver
    vio_node = Node(
        name='vio_node',
        package='slt_vio',
        executable='vio_node',
        parameters=[{'vio_config': vio_config}],
        output='screen',
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config],
        output='screen',
    )
    return LaunchDescription([vio_node, rviz2])
