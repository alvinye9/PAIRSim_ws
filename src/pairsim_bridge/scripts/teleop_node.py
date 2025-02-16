#!/usr/bin/env python3

# 
#   Copyright 2025 Alvin Ye
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions a  nd
#    limitations under the License.
# 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32, Int8
from autonoma_msgs.msg import VehicleInputs


from pynput import keyboard
class TeleopNode(Node):
    
    def __init__(self):
        super().__init__('teleop_node')
        
        self.cmd_input_publisher = self.create_publisher(
            VehicleInputs,
            '/vehicle_inputs',
            10
        )
        
        self.throttle_publisher = self.create_publisher(
            Float32,
            '/joystick/throttle_cmd',
            10
        )

        self.brake_publisher = self.create_publisher(
            Float32,
            '/joystick/brake_cmd',
            10
        )
        self.steer_publisher = self.create_publisher(
            Float32,
            '/joystick/steering_cmd',
            10
        )
        self.gear_publisher = self.create_publisher(
            Int8,
            '/joystick/gear_cmd',
            10
        )
               
        # self.timer = self.create_timer(0.005, self.publish_vehicle_inputs) #uncomment if you want to run teleop with AWSIM repub node
        self.timer = self.create_timer(0.005, self.publish_joystick) #uncomment if you want to run teleop with repub node
        
        self.curr_acc_cmd = 0.0
        self.curr_brake_cmd = 0.0
        self.curr_steer_cmd = 0.0
        self.curr_gear_cmd= 1
        
        listener = keyboard.Listener(on_press=self.on_press) #maybe add on-release
        listener.start()        


    def on_press(self, key):
        try:
            if key.char == 'w':
                self.curr_acc_cmd = min(self.curr_acc_cmd + 10.0, 100.0)  #[0-100]%
                self.curr_brake_cmd = 0.0
            elif key.char == 's':
                self.curr_acc_cmd = 0.0
                self.curr_brake_cmd = min(self.curr_brake_cmd + 10.0, 100.0)  
            elif key.char == 'a':
                self.curr_steer_cmd = min(self.curr_steer_cmd + 10.0, 240.0)  # Left
            elif key.char == 'd':
                self.curr_steer_cmd = max(self.curr_steer_cmd - 10.0, -240.0)  # Right
            elif key.char in '123456':
                self.curr_gear_cmd = int(key.char)
            else:
                print("Invalid Key Input: Use W [Throttle Up] A [Left Steer] S [Brake Up] D [Right Steer] and 123456 [Gear]")
        except AttributeError:
            print("Invalid Key Input: Use W [Throttle Up] A [Left Steer] S [Brake Up] D [Right Steer] and 123456 [Gear]")
            pass  # Special keys (ctrl, alt, etc.) will end up here
        print("Throttle: ", self.curr_acc_cmd , " Brake: ", self.curr_brake_cmd, " Steer: ", self.curr_steer_cmd, " Gear: ", self.curr_gear_cmd)
     
    def publish_vehicle_inputs(self):
        msg = VehicleInputs()
        msg.header = Header()

        msg.throttle_cmd = self.curr_acc_cmd # Percent
        msg.brake_cmd = self.curr_brake_cmd #kPa
        msg.steering_cmd = self.curr_steer_cmd #degrees
        msg.gear_cmd= self.curr_gear_cmd
        
        self.cmd_input_publisher.publish(msg)
        
    def publish_joystick(self):
        throttle_msg = Float32()
        brake_msg = Float32()
        steer_msg = Float32()
        gear_msg = Int8()
        throttle_msg.data = self.curr_acc_cmd 
        brake_msg.data = self.curr_brake_cmd 
        steer_msg.data = self.curr_steer_cmd / 15.0 #divide by 15.0 to compensate for repub nodes' steer constant
        gear_msg.data = self.curr_gear_cmd
        
        self.throttle_publisher.publish(throttle_msg)
        self.brake_publisher.publish(brake_msg)
        self.steer_publisher.publish(steer_msg)
        self.gear_publisher.publish(gear_msg)
        
        
def main(args=None):
    print("Running Teleop Node! (200 Hz)")
    rclpy.init(args=args)
    teleop_node = TeleopNode()

    try:
        rclpy.spin(teleop_node)
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()