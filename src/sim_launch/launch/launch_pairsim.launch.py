import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    map_arg = DeclareLaunchArgument('map', default_value='monza', 
            description='Map to run.  Options: monza, vegas, ims, putnam, lor')

    pkg = get_package_share_directory('planner_emulator')
    maps_config = os.path.join(pkg, 'config', 'maps.yaml')
    
    #Run repub node to republish various vehicle states and sensor data into autonomy stack and republish transform
    repub_node = Node(
        package='pairsim_bridge',
        executable='repub_node.py',
        output='screen',
        parameters=[
            
            os.path.join(get_package_share_directory('pairsim_bridge'), 'params', 'simulator.yaml'),
            maps_config,
            {'map': LaunchConfiguration('map')},
            ]
    )
    
    repub_ct_node = Node(
        package='pairsim_bridge',
        executable='repub_ct_node.py',
        output='screen',
    )
    
    return LaunchDescription([
        map_arg,
        repub_node,
        repub_ct_node,
    ])
