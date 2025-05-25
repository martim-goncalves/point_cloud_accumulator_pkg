import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('point_cloud_accumulator_pkg'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        *declare_parameters(),
        Node(
            package='point_cloud_accumulator_pkg',
            executable='point_cloud_accumulator_node',
            name='point_cloud_accumulator',
            output='screen',
            parameters=[
                config,
                config_dict()
            ],
            remappings=[
                ('/cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
                ('/camera/pose', '/zed/zed_node/pose_confidence'),
                ('/camera/loop_closure_event', '/zed/zed_node/loop_closure_detection_status')
            ]
        )
    ])

def declare_parameters():
    return [
        DeclareLaunchArgument('min_points_thr', default_value='10000'),
        DeclareLaunchArgument('max_points_thr', default_value='100000'),
        DeclareLaunchArgument('min_voxel_size_m', default_value='0.01'),
        DeclareLaunchArgument('max_voxel_size_m', default_value='0.1'),
        DeclareLaunchArgument('savefolder', default_value='./artifacts/'),
        DeclareLaunchArgument('savefile', default_value='accumulated_cloud'),
        DeclareLaunchArgument('save_interval_seconds', default_value='0'),
        DeclareLaunchArgument('enable_logging', default_value='true'),

        DeclareLaunchArgument('filters.transform.max_translation_m', default_value='0.3'),
        DeclareLaunchArgument('filters.transform.max_rotation_deg', default_value='15.0'),
        DeclareLaunchArgument('filters.transform.history_size', default_value='5'),

        DeclareLaunchArgument('filters.point.num_neighbors', default_value='20'),
        DeclareLaunchArgument('filters.point.dist_thr_m', default_value='0.02'),
        DeclareLaunchArgument('filters.point.min_appearance_ratio', default_value='0.6'),
        DeclareLaunchArgument('filters.point.min_neighbors', default_value='5'),
        DeclareLaunchArgument('filters.point.std_ratio', default_value='2.0'),
        DeclareLaunchArgument('filters.point.history_size', default_value='5'),

        DeclareLaunchArgument('filters.color.saturation_thr', default_value='240'),
        DeclareLaunchArgument('filters.color.history_size', default_value='10'),
    ]

def config_dict():
    return {
        'min_points_thr': LaunchConfiguration('min_points_thr'),
        'max_points_thr': LaunchConfiguration('max_points_thr'),
        'min_voxel_size_m': LaunchConfiguration('min_voxel_size_m'),
        'max_voxel_size_m': LaunchConfiguration('max_voxel_size_m'),
        'savefolder': LaunchConfiguration('savefolder'),
        'savefile': LaunchConfiguration('savefile'),
        'save_interval_seconds': LaunchConfiguration('save_interval_seconds'),
        'enable_logging': LaunchConfiguration('enable_logging'),

        'filters.transform.max_translation_m': LaunchConfiguration('filters.transform.max_translation_m'),
        'filters.transform.max_rotation_deg': LaunchConfiguration('filters.transform.max_rotation_deg'),
        'filters.transform.history_size': LaunchConfiguration('filters.transform.history_size'),

        'filters.point.num_neighbors': LaunchConfiguration('filters.point.num_neighbors'),
        'filters.point.dist_thr_m': LaunchConfiguration('filters.point.dist_thr_m'),
        'filters.point.min_appearance_ratio': LaunchConfiguration('filters.point.min_appearance_ratio'),
        'filters.point.min_neighbors': LaunchConfiguration('filters.point.min_neighbors'),
        'filters.point.std_ratio': LaunchConfiguration('filters.point.std_ratio'),
        'filters.point.history_size': LaunchConfiguration('filters.point.history_size'),

        'filters.color.saturation_thr': LaunchConfiguration('filters.color.saturation_thr'),
        'filters.color.history_size': LaunchConfiguration('filters.color.history_size'),         
    }
