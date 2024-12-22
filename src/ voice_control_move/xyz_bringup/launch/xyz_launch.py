import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription(
        [
            Node(                                           # 再创建一个节点
                package="xyz",                
                executable="xyz_node",
                name="xyz_node",
                parameters=[{'use_gpu': True}],             # 启用GPU
                output='screen',
                remappings=[
                    ('/input_topic', '/gpu_input_topic'),  # 示例重映射
                ],
            ),
        ]
    )