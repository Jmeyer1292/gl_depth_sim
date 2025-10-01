from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')
    mesh_filename = LaunchConfiguration('mesh_filename')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('gl_depth_sim'),
                'launch', 'laser_example.rviz'
            ),
            description='Path to the RViz config file'
        ),
        DeclareLaunchArgument(
            'mesh_filename',
            default_value=os.path.join(
                get_package_share_directory('gl_depth_sim'),
                'test', 'stanford_dragon.stl'
            ),
            description='Path to the mesh file'
        ),
        Node(
            package='gl_depth_sim',
            executable='laser_example',
            name='laser_example',
            output='screen',
            parameters=[{
                'mesh_filename': mesh_filename,
                'min_range': 0.1,
                'max_range': 30.0,
                'angular_resolution': 0.001
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
