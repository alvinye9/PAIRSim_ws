import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = False

    controller_arg = DeclareLaunchArgument('controller', default_value='pid',  
            description='Controller to run')

    
    logger_arg = DeclareLaunchArgument("log_level", default_value=["info"],
        description="Logging level")

    logger = LaunchConfiguration("log_level")

    kin_package = get_package_share_directory('kin_control')
    kin_config = kin_package + '/config/params.yaml'

    long_package = get_package_share_directory('long_control')
    long_config = long_package + '/config/params.yaml'

    accel_intf_package = get_package_share_directory('accel_intf')
    accel_config = accel_intf_package + '/config/cgs_param.yaml'

    launch_descr = []
    launch_descr.extend([controller_arg, logger_arg])

    adrc_package = get_package_share_directory("adrc_controller")
    adrc_config = adrc_package + '/params/ADRC_coupled_control.params.yaml'
    
    mpc_package = get_package_share_directory("fbl_mpc")
   

    #
    # PID
    #
    pid_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'pid'"]))

    long_control = Node(
        condition=pid_condition,
        package='long_control',
        executable='long_control',
        name='LongControlNode',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'mute': False},
            long_config
        ]
    )

    kin_control = Node(
        condition=pid_condition,
        package='kin_control',
        executable='kin_control',
        name='KinControlNode',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        #remappings=[('/joystick/steering_cmd', '/joystick/kin_control_steering_cmd')],
        parameters=[
            {'use_sim_time': False},
            {'mute': False},
            kin_config,
        ]
    )

    launch_descr.extend([long_control, kin_control])

    #
    # MPC
    #

    mpc_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'mpc'"]))

    mpc_control = Node(
        condition=mpc_condition,
        package='mpc',
        executable='MPC.py',
        name='MPC_Node',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    mpc_long_controller = Node(
        condition=mpc_condition,
        package='mpc',
        executable='long_mpc.py',
        name='Long_MPC_Node',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )

    mpc_accel_intf = Node(
        condition=mpc_condition,
        package='accel_intf',
        executable='accel_intf',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {'use_sim_time': use_sim_time},
            accel_config
        ]
    )

    launch_descr.extend([mpc_control, mpc_long_controller, mpc_accel_intf])

    #
    # Bicycle MPC
    #

    mpcb_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'mpcb'"]))
    
    #pairsim
    mpcb_condition_pair = IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'mpcb'"]))
   
    mpcb_control_pair = Node(
        condition=mpcb_condition_pair,
        package='fbl_mpc',
        executable='fbl_mpc.py',
        name='MPC_Node',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {'use_sim_time': use_sim_time},
            mpc_package + '/params/aw_params.yaml',
            # {"lateral_error_weight": 1.0}
   
      
        ]
    )

    mpcb_accel_intf = Node(
        condition=mpcb_condition,
        package='accel_intf',
        executable='accel_intf',
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {'use_sim_time': use_sim_time},
            accel_config
        ]
    )

    launch_descr.extend([mpcb_control_pair, mpcb_accel_intf])
    #
    # Fred Controller
    #
    fred_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'fred'"]))

    fred_node = Node(
    package='ferdinand_controller',
    executable='ferdinand_controller',
    output='screen',
    condition=fred_condition,
    parameters=[
        get_package_share_directory("ferdinand_controller") + '/params/ferdinand_oval.yaml',
        {"use_sim_time": use_sim_time},
    ],
    remappings=[
        ('/planning/offset_path','/planning/front_path/offset_path'),
    ],
    # prefix=["xterm -e gdb --args"],
    # prefix=["xterm -e gdb -ex run --args"], # Debugger
    )

    fred_accel_intf_node = Node(
        package="accel_intf",
        executable="accel_intf",
        output="screen",
        condition=fred_condition,
        parameters=[
            get_package_share_directory('accel_intf') + '/config/cgs_param.yaml',
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ('/control/controller/accel_command', 'control/ferdinand_controller/accel_command'),
            ('/control/controller/command', '/control/ferdinand_controller/command'),
        ],
    )
    launch_descr.extend([fred_node, fred_accel_intf_node])


    adrc_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'adrc"]))
    adrc_controller = Node(
        package='adrc_controller',
        executable='ADRC_coupled_control_node',
        output='screen',
        condition=adrc_condition,
        parameters=[
            {'use_sim_time': use_sim_time},
            adrc_config,
        ],
        remappings=[
            ('/planning/offset_path','/planning/front_path/offset_path'),
        ],
        #prefix=["xterm -e gdb --args"],
        #prefix=["xterm -e gdb -ex run --args"], # Debugger
    )

    adrc_accel_int = Node(
        package='accel_intf',
        executable='accel_intf',
        output='screen',
        condition=adrc_condition,
        arguments=['--ros-args', '--log-level', logger],
        parameters=[
            {'use_sim_time': use_sim_time},
            accel_config
        ]
    )
    # launch_descr.extend([adrc_controller, adrc_accel_int]) #FIXME, ADRC broken



    return LaunchDescription(launch_descr)

