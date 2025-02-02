import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


def generate_launch_description():
    
    map_arg = DeclareLaunchArgument('map', default_value='monza', 
            description='Map to run.  Options: monza, vegas, ims, putnam, lor')

    controller_arg = DeclareLaunchArgument('controller', default_value='pid',
            description='Controller to run.  Options: pid...')
    
    launch_controller_arg = DeclareLaunchArgument('launch_controller', default_value='true',
            description='Specify whether to run PAIRSIM with teleop or not')


    launch_descr = []

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sim_launch"), "launch", 
                "launch_planner.launch.py")
        ),
        launch_arguments={
            'map': LaunchConfiguration('map')}.items(),
    )
    
    controller_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('launch_controller'), "'.lower() in ['true']"]))
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sim_launch"), "launch", 
                "launch_controller.launch.py")
        ),
        launch_arguments={
            'controller': LaunchConfiguration('controller')}.items(),
        condition=controller_condition,
    )

    teleop_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('launch_controller'), "'.lower() in ['false']"]))
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sim_launch"), "launch", 
                "launch_teleop.launch.py")
        ),
        condition=teleop_condition,
    )
    
    pairsim_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("sim_launch"), "launch", 
                "launch_pairsim.launch.py")
        ),
        launch_arguments={
            'map': LaunchConfiguration('map')}.items(),
    )




    launch_descr.extend([
        ## Args:
        map_arg,
        controller_arg,
        # sim_arg,
        launch_controller_arg,
        ## Nodes:
        planner,
        teleop,
        controller,
        pairsim_simulator,

    ])

    return LaunchDescription(launch_descr)
