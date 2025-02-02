import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    
    #Run teleop node (default is publishing to /joystick not directly to AWSIM's /vehicle_inputs)
    teleop_node = Node(
        package='pairsim_bridge',
        executable='teleop_node.py',
        output='screen',
    )


    return LaunchDescription([
        teleop_node
    ])
