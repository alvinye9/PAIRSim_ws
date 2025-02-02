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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from deep_orange_msgs.msg import RcToCt
from autonoma_msgs.msg import ToRaptor, RaceControl
from std_msgs.msg import Float32, Int8, Header

class RepubCtNode(Node):
    #rc_to_ct msg
    rolling_counter = 0
    track_flag = 0
    vehicle_flag = 0
    sector_flag = 0
    lte_rssi = 0
    lte_sync_ok = False
    vehicle_rank = 0
    lap_count = 0
    lap_distance = 0.0
    round_target_speed = 0.0
    sys_state = 255
    ct_state = 255


    def __init__(self):
        super().__init__('repub_ct_node')

        self.subscription = self.create_subscription(
            RaceControl, 
            '/race_control', 
            self.race_control_callback, #pos, vel, attitude
            10)      

        self.ct_publisher = self.create_publisher(
            RcToCt,
            '/raptor_dbw_interface/rc_to_ct', 
            10)
        
        self.to_raptor_publisher = self.create_publisher(
            ToRaptor,
            '/to_raptor',
            10
        )
        
        # create timer to publish periodically
        PUBLISH_RATE = 0.01
        # self.timer1 = self.create_timer(PUBLISH_RATE, self.publish_ct)
        self.timer2 = self.create_timer(PUBLISH_RATE, self.publish_to_raptor)



    def race_control_callback(self, msg):
        self.track_flag = msg.track_flag
        self.vehicle_flag = msg.veh_flag
        self.lap_count = msg.lap_count
        self.lap_distance = msg.lap_distance
        self.sys_state = msg.sys_state
        self.round_target_speed = float(msg.round_target_speed)


    def publish_ct(self):
        # Create an RcToCt message
        ct_msg = RcToCt()
        # Fill in the header information
        ct_msg.stamp = self.get_clock().now().to_msg()
        self.rolling_counter = (self.rolling_counter + 1) % 256
        ct_msg.rolling_counter = self.rolling_counter 
        ct_msg.track_flag = self.track_flag
        ct_msg.vehicle_flag = self.vehicle_flag
        ct_msg.lap_count = self.lap_count
        ct_msg.lap_distance = self.lap_distance
        ct_msg.round_target_speed = self.round_target_speed

        # Publish the  message
        self.ct_publisher.publish(ct_msg) 

    def publish_to_raptor(self):
        msg = ToRaptor()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg() 
        msg.track_cond_ack = self.track_flag
        msg.veh_sig_ack = self.vehicle_flag

        msg.veh_num = 1 #FIXME: not used atm
        msg.rolling_counter = self.rolling_counter
        self.rolling_counter = (self.rolling_counter + 1) % 8
        
        if self.ct_state == 255:
            if self.track_flag == 1:
                self.ct_state = 1
                msg.ct_state = 1
                
        elif self.ct_state == 1:
            self.ct_state = 2
            msg.ct_state = 2
        
        elif self.ct_state == 2:
            if self.track_flag == 1:
                self.ct_state = 3
                msg.ct_state = 3
                
        elif self.ct_state == 3:
            if self.track_flag == 1:
                self.ct_state = 4
                msg.ct_state = 4 

        elif self.ct_state == 4:
            if self.track_flag == 2:
                self.ct_state = 5
                msg.ct_state = 5  
                
        elif self.ct_state == 5:
            if self.sys_state == 8:
                self.ct_state = 6
                msg.ct_state = 6  

        elif self.ct_state == 6:
            self.ct_state = 7
            msg.ct_state = 7 

        elif self.ct_state == 7:
            if self.sys_state == 9 and self.track_flag == 3:
                self.ct_state = 8
                msg.ct_state = 8 
                
        elif self.ct_state == 8:
            if self.track_flag == 4:
                self.ct_state = 9
                msg.ct_state = 9         
            
            
        
        self.to_raptor_publisher.publish(msg) 



def main(args=None):
    rclpy.init(args=args)
    node = RepubCtNode()

    # # Print a message to indicate that the message has been published
    # print("Message published once. Exiting...")
    print("publishing to rc_to_ct....")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()