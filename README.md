**How To Use**

This workspace contains tools to do rapid controller prototype development for PAIRSim

To get started, clone the repo:

`git clone git@github.itap.purdue.edu:Purdue-AI-Racing/simulator_ws.git --recurse-submodules`

Then install dependencies:
```
sudo apt install libgeographic-dev
```

Then, build the package, source the environment, and run the launch file:

```
cd simulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch sim_launch launch_sim.launch.py 
```


**Adding Maps**

Typically, a map is a CSV file containing, at minimum, X and Y coordinates.  Typically, velocity and curvature are also included.  This map can be generated from several sources, including https://github.com/TUMFTM/global_racetrajectory_optimization.

Once a CSV file is obtained, place it in the `/simulator_ws/src/planner_emulator/maps` directory.  Don't forget to re-build and re-source your workspace after adding new files.  
Next, add an entry to `/simulator_ws/src/planner_emulator/config/maps.yaml`.  `csv_file` should be the name of the csv file in the maps directory.  `initial_pos` should be the initial position in the form [x, y, psi], where psi is the heading.
`col_order` describes the structure of your csv file.  List the 0-based index of the columns in the order `[X Coordinate, Y Coordinate, Velocity, Curvature]`.  For example:

```
lvms:
    csv_file: 'vegas_centerline.csv' 
    init_pos: [0.0, 0.0, -2.44346]
    col_order: [0, 1, 2, 3]  
```

Then, you can pass the map launch argument to the launch_sim launch file:

`ros2 launch sim_launch launch_sim.launch.py map:=lvms`

The `csv_file` and `col_order` parameters are passed to the planner emulator

**Adding Controllers**

To add a controller, first place your code in the src directory.  Then, you will need to add a portion to the file `/simulator_ws/src/sim_launch/launch/launch_controller.launch.py` that looks similar to the following:

```
 41     #
 42     # PID
 43     #
 44     pid_condition = IfCondition(PythonExpression(["'", LaunchConfiguration('controller'), "' == 'pid'"]))
 45 
 46     long_control = Node(
 47         condition=pid_condition,
 48         package='long_control',
 49         executable='long_control',
 50         name='LongControlNode',
 51         output='screen',
 52         arguments=['--ros-args', '--log-level', logger],
 53         parameters=[
 54             {'use_sim_time': use_sim_time},
 55             {'mute': False},
 56             long_config
 57         ]
 58     )
 59 
 60     kin_control = Node(
 61         condition=pid_condition,
 62         package='kin_control',
 63         executable='kin_control',
 64         name='KinControlNode',
 65         output='screen',
 66         arguments=['--ros-args', '--log-level', logger],
 67         #remappings=[('/joystick/steering_cmd', '/joystick/kin_control_steering_cmd')],
 68         parameters=[
 69             {'use_sim_time': False},
 70             {'mute': False},
 71             kin_config,
 72         ]
 73     )
 74 
 75     launch_descr.extend([long_control, kin_control])
```

Start by replacing 'pid' with your controller name in line 44.  This line serves to dynamically enable or disable your controller based on launch arguments.  Next, add your control strategy's required nodes,
making sure to include the `condition=my_awesome_condition` line in each one.  Then, add the conditional nodes to the launch description list as in line 75.

Finally, add your node as exec depend in the `sim_launch/package.xml` file.

To launch your controller, use:

`ros2 launch sim_launch launch_sim.launch.py controller:=pid`


**Specifying Parameters via Command Line**

<!-- For more instructions on navigating the PAIRSIM GUI, see Step 3 https://github.itap.purdue.edu/Purdue-AI-Racing/on-vehicle/wiki/Running-PAIRSIM-with-on%E2%80%90vehicle -->

map:= [options: monza, lvms, ims, kentucky, lor]

controller:= [recommended options: pid, mpcb]

launch_controller:=false (to disable controller and launch teleop instead)

