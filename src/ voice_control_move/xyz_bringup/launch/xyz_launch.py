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
    # 包含另一个启动文件的描述
    demo_nodes = IncludeLaunchDescription(
        # 指定要包含的启动文件的路径
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("whisper_bringup"), "launch"),
                 "/whisper.launch.py",
            ]
        ),
    )

    # 返回一个启动描述对象，包含两个节点
    return LaunchDescription(
        [
            demo_nodes,                                     # 前面指定的 IncludeLaunchDescription
            Node(                                           # 再创建一个节点
                package="whisper_demos",                
                executable="whisper_demo_node",
                name="whisper_demo_node",
                parameters=[{'use_gpu': True}],             # 启用GPU
                output='screen',
                remappings=[
                    ('/input_topic', '/gpu_input_topic'),  # 示例重映射
                ],
            )
        ]
    )