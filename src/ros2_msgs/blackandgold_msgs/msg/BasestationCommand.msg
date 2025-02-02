float32 desired_speed                 # how fast we want to go
float32 exp_float                     # unused
string description                    # unused
bool kin_enable                       # whether auto steering is enabled
bool long_enable                      # whether auto throttle/brake is enabled
bool graceful_stop                    # set to true to trigger graceful stop
bool emergency_stop                   # set to true to trigger emergency stop
bool enable_overtaking                # are we allowed to overtake
bool take_pit_trajectory              # set to true to pit
bool enable_engine                    # set to true to start engine, set to false to stop engine
bool partial_safety_override_enabled  # set to true to disable blacklisted checks from the nodes and modules specified in safety_node.cs
bool complete_safety_override_enabled # set to true to disable all but whitelisted checks from the nodes and modules specified in safety_node.cs
bool clear_errors                     # set to true to clear all errors picked up by the safety_module (This is dangerous!)
string desired_controller             # used to set the controller to use
string set_param_command              # command used to set parameters on the fly

bool spawn_ghost_vehicle              # set to true to spawn, set to false to stop publishing
float32 ghost_vehicle_velocity        # the desired velocity of the ghost vehicle
float32 ghost_vehicle_spawn_distance  # the desired spawn distance of the ghost vehicle
float32 ghost_vehicle_offset          # lateral offset for the offset

# DEPRECATED 
bool enable_safety_override