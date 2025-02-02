import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pkg = get_package_share_directory('planner_emulator')

    # Launch Arguments
    map_arg = DeclareLaunchArgument('map', default_value="monza", 
                    description="Map to run from maps config")
    map_dir_arg = DeclareLaunchArgument('map_dir', default_value=os.path.join(pkg,'maps'), 
                    description="Directory that maps are stored in")
    sim_time_arg = DeclareLaunchArgument('sim_time', default_value="false", 
                    description='Use sim time')

    maps_config = os.path.join(pkg, 'config', 'maps.yaml')


    # Run transform_path.py script node
    path_publisher = Node(
        package='planner_emulator',
        executable='transform_path.py',
        output='screen',
        parameters=[
            maps_config,
            {'map': LaunchConfiguration('map')},
            {'map_dir': LaunchConfiguration('map_dir')},
            {'use_sim_time': LaunchConfiguration('sim_time')}
        ]
    )
    
    global_path_publisher = Node(
        package='planner_emulator',
        executable='global_path_publisher.py',
        output='screen',
        parameters=[
            maps_config,
            {'map': LaunchConfiguration('map')},
            {'map_dir': LaunchConfiguration('map_dir')},
            {'use_sim_time': LaunchConfiguration('sim_time')}
        ]
    )
    return LaunchDescription([
        # Args:
        map_arg,
        map_dir_arg,
        sim_time_arg,
        # Nodes:
        path_publisher,
        global_path_publisher,
    ])
