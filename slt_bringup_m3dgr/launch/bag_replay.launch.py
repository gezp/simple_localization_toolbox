import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slt_bringup_m3dgr = get_package_share_directory('slt_bringup_m3dgr')
    rviz2_config = os.path.join(pkg_slt_bringup_m3dgr, 'launch', 'bag_replay.rviz')
    data_dir = os.path.join(
        os.environ.get('HOME', '.'), 'localization_data', 'dataset', 'M3DGR', 'Standard'
    )
    rosbag_node = ExecuteProcess(
        name='rosbag',
        cmd=[
            'ros2 bag play',
            LaunchConfiguration('bag_path'),
            '-r',
            LaunchConfiguration('rate'),
            '-d',
            '3',
            '--read-ahead-queue-size',
            '1000',
        ],
        shell=True,
        output='screen',
    )
    calibration_config = os.path.join(
        pkg_slt_bringup_m3dgr, 'config', 'calibration.yaml'
    )
    replay_node = Node(
        package='slt_bringup_m3dgr',
        executable='replay_node',
        name='replay_node',
        output='screen',
        parameters=[
            {
                'calibration_config': calibration_config,
                'gt_path': LaunchConfiguration('gt_path'),
                'rate': LaunchConfiguration('rate'),
                'publish_tf': True,
                'rebuild_gt_rotation': LaunchConfiguration('rebuild_gt_rotation'),
            }
        ],
    )
    rviz2 = ExecuteProcess(
        cmd=['rviz2', '-d', rviz2_config],
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
            DeclareLaunchArgument('rebuild_gt_rotation', default_value='true'),
            rosbag_node,
            replay_node,
            rviz2,
        ]
    )
