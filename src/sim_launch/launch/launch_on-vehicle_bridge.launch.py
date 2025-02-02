import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    repub_ct_node = Node(
        package='pairsim_bridge',
        executable='repub_ct_node.py',
        output='screen',
    )
    
    repub_ekf_node = Node(
        package='pairsim_bridge',
        executable='repub_ekf_node.py',
        output='screen',
    )
    
    return LaunchDescription([
        repub_ct_node,
        repub_ekf_node,
    ])
    
