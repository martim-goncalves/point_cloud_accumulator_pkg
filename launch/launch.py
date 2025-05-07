import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('point_cloud_accumulator_pkg'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='point_cloud_accumulator_pkg',
            executable='point_cloud_accumulator_node',
            name='point_cloud_accumulator',
            output='screen',
            parameters=[config],
            remappings=[
                ('/cloud_in', '/zed/zed_node/point_cloud/cloud_registered')
            ]
        )
    ])
