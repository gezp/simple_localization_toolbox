import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_slt_bringup_m3dgr = get_package_share_directory('slt_bringup_m3dgr')
    rviz2_config = os.path.join(pkg_slt_bringup_m3dgr, 'launch', 'lidar_odometry.rviz')
    lidar_odometry_config = os.path.join(
        pkg_slt_bringup_m3dgr, 'config', 'lidar_odometry.yaml'
    )
    calibration_config = os.path.join(
        pkg_slt_bringup_m3dgr, 'config', 'calibration.yaml'
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
    lidar_odometry_node = Node(
        name='lidar_odometry_node',
        package='slt_lidar_odometry',
        executable='lidar_odometry_node',
        parameters=[
            {
                'lidar_odometry_config': lidar_odometry_config,
                'undistort_point_cloud': False,
                'publish_undistorted_point_cloud': False,
                'use_initial_pose_from_topic': False,
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'lidar_frame_id': 'livox_avia_lidar',
                'odom_frame_id': 'odom_lidar',
            }
        ],
        remappings=[
            ('synced_cloud', '/livox/avia/points'),
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
                    os.environ.get('HOME', '.'), 'localization_data', 'trajectory'
                ),
                'odom_names': ['ground_truth', 'odom_lidar'],
                'odom_topics': ['m3dgr/ground_truth/odom', 'lidar_odometry/odom'],
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
            lidar_odometry_node,
            simple_evaluator_node,
            rviz2,
        ]
    )
