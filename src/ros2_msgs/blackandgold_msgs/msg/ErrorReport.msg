#  Copyright 2023-2024 Alec Pannunzio <apun189@gmail.com>
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License. 


# Message type for reporting faults on /<modulename>/errors

# standard fields
std_msgs/Header header
string description   # description of the error
int8 severity        # what is the severity of this error? Should be one of the ERROR_ constants below.
string origin        # where this error is coming from (usually just node name)
string module        # What module is this from? This should match the namespace of the topic you are publishing to! if module == planning, topic should be published to /planning/errors!
float32 lifetime 1.0 # how long this error should persist (in seconds), -1 is infinite

# advanced fields (should normally be left default)
bool engine_shutdown False        # whether this error requires shutting the engine down
float32 error_speed_override -1.0 # overrides the default speed for this error level

# constant declarations
# -----------------------------------

# error level constants
int8 ERROR_ALL_CLEAR=0           # There are no issues
int8 ERROR_WARNING=1             # There may be issues, but they are not critical
int8 ERROR_SOFT_FAULT=2          # There is a minor problem. Limit velocity to a comfortable speed
int8 ERROR_FAULT=3               # There is a problem. Disable overtaking and limit speed greatly
int8 ERROR_CRITICAL=4            # There is a critical error. Limit speed extremely
int8 ERROR_FATAL=5               # Fatal error. Stop the car
int8 ERROR_EMERGENCY_STOP=6      # Emergency stop, locks the breaks and manually zeroes throttle. should only be used on dead controller.
int8 MAX_ERROR_LEVEL=6           # Should always be updated to the level of the highest error level

# speed level constants
float32 SPEED_WARNING=1000.0     # max speed for error level 1
float32 SPEED_SOFT_FAULT=1000.0  # max speed for error level 2
float32 SPEED_FAULT=10.0         # max speed for error level 3
float32 SPEED_CRITICAL=3.0       # max speed for error level 4
float32 SPEED_FATAL=0.0          # max speed for error level 5
float32 MIN_ACCEPTABLE_DECEL=0.5 # minimum allowable deceleration while approaching an error velocity
float32 ERROR_SPEED_MARGIN=2.0   # margin of error allowed for approaching error speeds