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
from std_msgs.msg import Float32, Int8, Header
from novatel_oem7_msgs.msg import BESTPOS, HEADING2, BESTVEL, INSPVA
from autonoma_msgs.msg import VehicleInputs, PowertrainData, VehicleData, ToRaptor
from deep_orange_msgs.msg import PtReport
from raptor_dbw_msgs.msg import WheelSpeedReport
from geometry_msgs.msg import PointStamped

from pyproj import CRS, Transformer 

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class RepubNode(Node):
    
    def __init__(self):
        
        super().__init__('repub_node')
        
        map_name = self.declare_parameter('map', 'monza').value
        
        ##================================================
        ## Monza Origin Values UTM 32N 
        ## Both the lat/lon and UTM coordinates are the origin of the car in AWSIM as well as the origin of where the ref line was generated WRT
        ## ENTER THESE VALUES INTO AWSIM FOR SKIDPAD (LAT,LON,HEIGHT,YAW): 45.618974079378670, 9.281181751068655, 177 (height optional), 0.0
        ##================================================
        
        if(map_name == 'monza'):
            self.EAST_ORIGIN = 521921.53 
            self.NORTH_ORIGIN = 5051752.75
            self.UTM_ZONE = 32
            self.UTM_ZONE_DIR = 'N'

        ##================================================
        ## LVMS Centerline Origin Values UTM 11N
        ## Both the lat/lon and UTM coordinates are the origin of the car in AWSIM as well as the origin of where the ref line was generated WRT
        ## ENTER THESE VALUES INTO AWSIM FOR SKIDPAD (LAT,LON,HEIGHT,YAW): 36.274499, -115.012626, 569 (height optional), 230 deg NED
        ##================================================            
        elif(map_name == 'lvms'):
            self.EAST_ORIGIN = 678501.0  #utm origin of car, a little bit behind the start of the ref line
            self.NORTH_ORIGIN = 4016225.0
            self.UTM_ZONE = 11
            self.UTM_ZONE_DIR = 'N'
            
        ##================================================
        ## LOR Centerline Origin Values UTM 16N
        ## Both the lat/lon and UTM coordinates are the origin of the car in AWSIM as well as the origin of where the ref line was generated WRT
        ## ENTER THESE VALUES INTO AWSIM FOR SKIDPAD (LAT,LON,HEIGHT,YAW): 39.812524, -86.341826, 261 (height optional), 180 deg NED
        ##================================================            
        elif(map_name == 'lor'):
            self.EAST_ORIGIN = 556334.0
            self.NORTH_ORIGIN = 4407155.0
            self.UTM_ZONE = 16
            self.UTM_ZONE_DIR = 'N'
            
        ##================================================
        ## Kentucky Speedway Centerline Origin Values UTM 16N
        ## Both the lat/lon and UTM coordinates are the origin of the car in AWSIM as well as the origin of where the ref line was generated WRT
        ## ENTER THESE VALUES INTO PAIRSIM FOR SKIDPAD (LAT,LON,HEIGHT,YAW): 38.712574, -84.918558, 223 (height optional), 215 NED
        ##================================================            
        elif(map_name == 'kentucky'):
            self.EAST_ORIGIN = 680972.0
            self.NORTH_ORIGIN = 4286938.0
            self.UTM_ZONE = 16
            self.UTM_ZONE_DIR = 'N'
       ##================================================
        ## IMS Centerline Origin Values UTM 16N
        ## Both the lat/lon and UTM coordinates are the origin of the car in AWSIM as well as the origin of where the ref line was generated WRT
        ## ENTER THESE VALUES INTO PAIRSIM FOR SKIDPAD (LAT,LON,HEIGHT,YAW): 39.793154, -86.238866, 219 (height optional), 180 NED
        ##================================================            
        elif(map_name == 'ims'):
            self.EAST_ORIGIN = 565166.13
            self.NORTH_ORIGIN = 4405076.78
            self.UTM_ZONE = 16
            self.UTM_ZONE_DIR = 'N'
        
        ##================================================
        ## Origin Values Corresponding to 0,0 lat/lon
        ##================================================
        else:
            self.EAST_ORIGIN = 166021.44
            self.NORTH_ORIGIN = 0.0
            self.UTM_ZONE = 31
            self.UTM_ZONE_DIR = 'N'
            

        self.STEER_ANGLE_RATIO = 15.0
        self.true_easting = self.EAST_ORIGIN 
        self.true_northing =  self.NORTH_ORIGIN 
        self.true_heading = 0.0 #degrees
        self.true_local_x = 0.0
        self.true_local_y = 0.0


        self.curr_acc_cmd = 0.0
        self.curr_brake_cmd = 0.0
        self.curr_steer_cmd = 0.0
        self.curr_gear_cmd= 0

        self.throttle_counter = 0
        self.brake_counter = 0
        self.steering_counter = 0
        self.gear_counter = 0
        self.rolling_counter = 0

        self.engine_rpm = 0.0
        self.current_gear = 0

        self.STEER_CONSTANT = 15.0 
        self.true_vel = 0.0
        
        self.ws_fl = 0.0
        self.ws_fr = 0.0
        self.ws_rl = 0.0
        self.ws_rr = 0.0       
        
        self.throttle_cmd_count = 0
        self.steering_cmd_count = 0
        self.brake_cmd_count = 0

        self.tf_broadcaster = TransformBroadcaster(self)
        
        #====================== Subscribers ======================
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/throttle_cmd',
            self.accelerator_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/brake_cmd',
            self.brake_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Float32,
            '/joystick/steering_cmd',
            self.steering_cmd_callback,
            10
        )
        self.subscription = self.create_subscription(
            Int8,
            '/joystick/gear_cmd',
            self.gear_cmd_callback,
            10
        )
        
        #====================== PAIRSIM Subscribers ======================
        self.subscription = self.create_subscription(
            PowertrainData,
            '/powertrain_data',
            self.powertrain_callback,
            10
        )
        self.subscription = self.create_subscription(
            BESTVEL, 
            '/COM/bestvel', 
            self.vel_callback, 
            10)
        self.subscription = self.create_subscription(
            INSPVA, 
            '/COM/inspva', 
            self.pos_heading_callback, 
            10)
        self.subscription = self.create_subscription(
            VehicleData, 
            '/vehicle_data', 
            self.ws_callback, #fr fl rr rl wheel speeds
            10)
        self.subscription = self.create_subscription(
            PowertrainData,
            '/powertrain_data',
            self.powertrain_callback,
            10)
        self.subscription = self.create_subscription(
            VehicleData,
            '/vehicle_data',
            self.ws_callback,
            10)

        #====================== Publishers ======================
        self.cmd_input_publisher = self.create_publisher(
            VehicleInputs,
            '/vehicle_inputs',
            10
        )
        self.vel_publisher = self.create_publisher(
            Float32,
            '/localization/vehicle_speed', 
            10)
        self.engine_rpm_publisher = self.create_publisher(
            PtReport,
            '/raptor_dbw_interface/pt_report',
            10)
        self.wheel_speed_report_publisher = self.create_publisher(
            WheelSpeedReport,
            '/raptor_dbw_interface/wheel_speed_report', 
            10)
        self.local_pos_publisher = self.create_publisher(
            PointStamped,
            '/ego_vehicle/position',
            10)

        self.timer1 = self.create_timer(0.01, self.publish_vehicle_inputs)
        self.timer2 = self.create_timer(0.01, self.publish_tf)
        self.timer3 = self.create_timer(0.01, self.publish_vel)
        self.timer4 = self.create_timer(0.01, self.publish_engine_rpm)
        self.timer5 = self.create_timer(0.01, self.publish_wheel_speed)
        self.timer6 = self.create_timer(0.01, self.publish_local_pos)
        
        

    def ws_callback(self,msg):
        self.ws_fl = msg.ws_front_left
        self.ws_fr = msg.ws_front_right
        self.ws_rl = msg.ws_rear_left
        self.ws_rr = msg.ws_rear_right
                
    def powertrain_callback(self, msg):
        self.engine_rpm = msg.engine_rpm
        self.current_gear = msg.current_gear

    def accelerator_cmd_callback(self, msg):
        self.curr_acc_cmd = msg.data

    def brake_cmd_callback(self,msg):
        self.curr_brake_cmd = msg.data

    def steering_cmd_callback(self,msg):
        steer_cmd = msg.data * self.STEER_ANGLE_RATIO
        self.curr_steer_cmd = steer_cmd  #make sure to include steer ratio to map front wheel angle from controller to steering wheel angle in AWSIM

    def gear_cmd_callback(self,msg):
        self.curr_gear_cmd= msg.data


    def vel_callback(self,msg):
        self.true_vel = msg.hor_speed #m/s
        #self.get_logger().info('Got velocity (m/s):' + str(self.true_vel))


    def pos_heading_callback(self, msg):
        # self.true_heading =  -1.0 * self.degrees_to_radians(msg.azimuth) - math.pi/2
        self.true_heading = math.radians( (-1.0 * msg.azimuth) + 90.0)
        lat = msg.latitude
        long = msg.longitude
        [self.true_easting, self.true_northing] = self.latlon_to_utm(lat, long)
        self.true_local_x = self.true_easting - self.EAST_ORIGIN
        self.true_local_y = self.true_northing - self.NORTH_ORIGIN
        
    #callback for timer 1
    def publish_vehicle_inputs(self):
        msg = VehicleInputs()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Adds the current timestamp

        msg.throttle_cmd = self.curr_acc_cmd # Percent    
        msg.brake_cmd = self.curr_brake_cmd #kPa
        msg.steering_cmd = self.curr_steer_cmd #degrees 
        msg.gear_cmd= self.curr_gear_cmd
        

        msg.throttle_cmd_count = self.throttle_cmd_count
        msg.brake_cmd_count = self.brake_cmd_count
        msg.steering_cmd_count = self.steering_cmd_count
        
        self.throttle_cmd_count = (self.throttle_cmd_count + 1) % 8  # Wrap around at 8
        self.brake_cmd_count = (self.brake_cmd_count + 1) % 8  # Wrap around at 8
        self.steering_cmd_count = (self.steering_cmd_count + 1) % 8  # Wrap around at 8
        
        # self.get_logger().info('throttle cmd count:' + str(self.throttle_cmd_count))
        # self.get_logger().info('throttle %:' + str(msg.throttle_cmd))
        # self.get_logger().info('brake kPa:' + str(msg.brake_cmd))
        # self.get_logger().info('steering degrees:' + str(msg.steering_cmd))
        # self.get_logger().info('gear:' + str(msg.gear_cmd))
        # self.get_logger().info('Republished vehicle cmds')
        
        self.cmd_input_publisher.publish(msg)
        

    # def publish_to_raptor(self):
    #     msg = ToRaptor()
    #     msg.header = Header()
    #     msg.header.stamp = self.get_clock().now().to_msg() 
    #     msg.track_cond_ack = 3
    #     msg.veh_sig_ack = 1
    #     msg.ct_state = 8
    #     msg.veh_num = 1
    #     msg.rolling_counter = self.rolling_counter
    #     self.rolling_counter = (self.rolling_counter + 1) % 8
        
    #     self.to_raptor_publisher.publish(msg)
        
        
    #callback for timer 2
    def publish_tf(self):
        # Replace these with your actual x, y positions, and yaw angle
        x = self.true_local_x
        y = self.true_local_y
        yaw = self.true_heading  # Yaw angle in radians
        
        t = TransformStamped()
        #assign values to corr. tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vehicle'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0  # Assuming a 2D scenario
        q = self.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        #self.get_logger().info('Rotation +Z:' + str(q[2]))
        self.tf_broadcaster.sendTransform(t)

    #callback for timer 3
    def publish_vel(self):
        vel_msg = Float32()
        vel_msg.data = self.true_vel
        self.vel_publisher.publish(vel_msg)
    
    def publish_engine_rpm(self):
        engine_rpm_msg = PtReport()
        engine_rpm_msg.engine_rpm = self.engine_rpm
        engine_rpm_msg.current_gear = self.current_gear
        engine_rpm_msg.engine_on_status = True
        self.engine_rpm_publisher.publish(engine_rpm_msg)

    def publish_wheel_speed(self):
        wheel_speed_msg = WheelSpeedReport()
        wheel_speed_msg.header.stamp = self.get_clock().now().to_msg()
        wheel_speed_msg.front_left = self.ws_fl
        wheel_speed_msg.front_right = self.ws_fr
        wheel_speed_msg.rear_left = self.ws_rl
        wheel_speed_msg.rear_right = self.ws_rr
        self.wheel_speed_report_publisher.publish(wheel_speed_msg)
        
    def publish_local_pos(self):
        pos_msg = PointStamped() #current position of static ghost obstable at 0,0
        pos_msg.point.x = self.true_local_x
        pos_msg.point.y = self.true_local_y
        self.local_pos_publisher.publish(pos_msg)

# ============== Helper Functions ================

    def latlon_to_utm(self, latitude, longitude):
        # Define UTM projection for a specific ZONE
        utm_zone = self.UTM_ZONE
        utm_band = self.UTM_ZONE_DIR  # Northern Hemisphere
        utm_proj = CRS.from_string(f'EPSG:{32600 + utm_zone}')
        
        transformer = Transformer.from_crs(CRS.from_epsg(4326), utm_proj, always_xy=True)
        # Transform latitude and longitude to UTM coordinates
        utm_easting, utm_northing = transformer.transform(longitude, latitude)

        return utm_easting, utm_northing

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

def main(args=None):
    rclpy.init(args=args)
    node = RepubNode()
    print("republishing...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
