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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Quaternion
from sensor_msgs.msg import Imu, NavSatFix
from novatel_oem7_msgs.msg import  INSPVA, BESTVEL, HEADING2
from rosgraph_msgs.msg import Clock
from pyproj import CRS, Transformer 
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class RepubEkfNode(Node):
    def __init__(self):
        super().__init__('repub_ekf_node')
        
    # Position
        self.true_easting = 0.0
        self.true_northing = 0.0
        self.true_heading = 0.0 # degrees
        self.true_local_x = 0.0
        self.true_local_y = 0.0
        self.latitude = 0.0
        self.longitude = 0.0
        self.height = 0.0

    # Orientation (Quarternion)
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.w = 0.0
        
    # Linear Velocity (in ego-vehicle's local frame)
        self.v_x = 0.0 
        self.v_y = 0.0
        self.v_z = 0.0
        self.v_east= 0.0
        self.v_north= 0.0

    # Angular Velocity
        self.v_roll = 0.0
        self.v_pitch = 0.0
        self.v_yaw = 0.0

    #Linear Acceleration
        self.a_x = 0.0
        self.a_y = 0.0
        self.a_z = 0.0

    #Imu data
        self.imu_data = Imu()

    ## For Novatel_Bottom        
    # Position
        self.true_easting_bottom = 0.0
        self.true_northing_bottom = 0.0
        self.true_heading_bottom = 0.0 # degrees
        self.ENU_heading = 0.0
        
        self.true_local_x_bottom = 0.0
        self.true_local_y_bottom = 0.0
        self.latitude_bottom = 0.0
        self.longitude_bottom = 0.0
        self.height_bottom = 0.0

    # Orientation (Quarternion)
        self.qx_bottom = 0.0
        self.qy_bottom = 0.0
        self.qz_bottom = 0.0
        self.w_bottom = 0.0
        
    # Linear Velocity
        self.v_x_bottom = 0.0
        self.v_y_bottom = 0.0
        self.v_z_bottom = 0.0
        self.v_east_bottom = 0.0
        self.v_north_bottom = 0.0

    # Angular Velocity
        self.v_roll_bottom = 0.0
        self.v_pitch_bottom = 0.0
        self.v_yaw_bottom = 0.0

    #Linear Acceleration
        self.a_x_bottom = 0.0
        self.a_y_bottom = 0.0
        self.a_z_bottom = 0.0

    #Imu data
        self.imu_data_bottom = Imu()        


      ### === SUBSCRIBERS        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            INSPVA, 
            '/novatel_top/inspva', 
            self.top_pva_callback, #pos, vel, attitude
            10)

        self.subscription = self.create_subscription( 
            INSPVA, 
            '/novatel_bottom/inspva', 
            self.bottom_pva_callback, #pos, vel, attitude
            10)
        
        self.subscription = self.create_subscription(
            Imu, 
            # '/novatel_top/rawimux', 
            '/novatel_top/imu/data_raw',
            self.top_imu_callback, #pos, vel, attitude
            10)
        
        self.subscription = self.create_subscription(
            Imu, 
            # '/novatel_bottom/rawimux', 
            '/novatel_bottom/imu/data_raw',
            self.bottom_imu_callback, #pos, vel, attitude
            10)

        self.subscription = self.create_subscription(
            HEADING2, 
            '/novatel_top/heading2', 
            self.top_heading_callback, #pos, vel, attitude
            10)

        ### === ODOM PUBLISHERS
        self.top_odom_publisher = self.create_publisher(
            Odometry,
            'novatel_top/odom',
            10
        )
        self.bottom_odom_publisher = self.create_publisher(
            Odometry,
            'novatel_bottom/odom',
            10
        )
    
         ## ==== CLOCK PUBLISHER ===
        self.clock_msg = Clock()
        self.clock_spoofer = self.create_publisher(
            Clock,
            "/clock",
            1)
        
        ## ==== IMU PUBLISHERS === 
        self.top_imu_data_publisher = self.create_publisher(
            Imu, 
            '/novatel_top/imu/data', 
            10)
        self.bottom_imu_data_publisher = self.create_publisher(
            Imu, 
            '/novatel_bottom/imu/data', 
            10)
        
        self.top_imu_data_raw_publisher = self.create_publisher(
            Imu, 
            '/novatel_top/imu/data_raw', 
            10)
        self.bottom_imu_data_raw_publisher = self.create_publisher(
            Imu, 
            '/novatel_bottom/imu/data_raw', 
            10)
        
        ## ==== FIX PUBLISHERS ===
        self.top_fix_publisher = self.create_publisher(
            NavSatFix, 
            '/novatel_top/fix', 
            10)
        self.bottom_fix_publisher = self.create_publisher(
            NavSatFix, 
            '/novatel_bottom/fix', 
            10)

        # # ==== TWIST PUBLISHERS ===
        self.top_ins_twist_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/novatel_top/ins_twist', 
            10)
        self.bottom_ins_twist_publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            '/novatel_bottom/ins_twist', 
            10)

        # create timer to publish 
        PUBLISH_RATE = 0.005
        self.clock_spoof_timer = self.create_timer(0.005, self.publish_clock)
        self.timer1 = self.create_timer(PUBLISH_RATE, self.publish_odometry)
        self.timer2 = self.create_timer(PUBLISH_RATE, self.publish_ins_twist)
        self.timer3 = self.create_timer(PUBLISH_RATE, self.publish_imu_data)
        self.timer4 = self.create_timer(PUBLISH_RATE, self.publish_fix)

    def top_pva_callback(self, msg):
        # Position (Latitude -> UTM -> Local x y)
        self.latitude  = msg.latitude
        self.longitude = msg.longitude
        self.height = msg.height
        [self.true_easting, self.true_northing] = self.latlon_to_utm(self.latitude, self.longitude)
        
        # # Orientation (Euler (radians) -> Quarternion) (azimuth is 0 facing north (NED), need to convert to ENU)
        self.true_heading = math.radians( ( -1.0 * msg.azimuth) + 90.0) #should be 90 at azimuth = 0, this governs the direction of base_link    
        roll_rad = math.radians(msg.roll)
        pitch_rad = math.radians(msg.pitch)
    
        q = self.get_quaternion_from_euler(roll_rad, pitch_rad, self.true_heading)
        self.qx = q[0]
        self.qy = q[1]
        self.qz = q[2]
        self.w = q[3]

        # # Linear Velocity
        self.v_east = msg.east_velocity
        self.v_north = msg.north_velocity
        self.v_x, self.v_y = self.calculate_local_velocity(self.true_heading, msg.east_velocity, msg.north_velocity)

    def bottom_pva_callback(self, msg):
        # Position (Latitude -> UTM -> Local x y)
        self.latitude_bottom  = msg.latitude
        self.longitude_bottom = msg.longitude
        self.height_bottom = msg.height
        [self.true_easting_bottom, self.true_northing_bottom] = self.latlon_to_utm(self.latitude_bottom, self.longitude_bottom)
        
        # # Orientation (Euler (radians) -> Quarternion) (azimuth is 0 facing north (NED), need to convert to ENU)
        self.true_heading_bottom = math.radians( ( -1.0 * msg.azimuth) + 90.0)     
        roll_rad = math.radians(msg.roll)
        pitch_rad = math.radians(msg.pitch)
    
        q = self.get_quaternion_from_euler(roll_rad, pitch_rad, self.true_heading) 
        self.qx_bottom = q[0]
        self.qy_bottom = q[1]
        self.qz_bottom = q[2]
        self.w_bottom = q[3]

        # # Linear Velocity
        self.v_east_bottom = msg.east_velocity
        self.v_north_bottom = msg.north_velocity
        self.v_x_bottom, self.v_y_bottom = self.calculate_local_velocity(self.true_heading_bottom, msg.east_velocity, msg.north_velocity)

    def top_imu_callback(self, msg):
        #recieve imu data
        self.imu_data = msg
        # angular velocity (if not directly repubing as Imu msg type)
        self.v_roll = msg.angular_velocity.x
        self.v_pitch = msg.angular_velocity.y
        self.v_yaw = msg.angular_velocity.z
        
    def bottom_imu_callback(self, msg):
        self.imu_data_bottom = msg
        # angular velocity (if not directly repubing as Imu msg type)
        self.v_roll_bottom = msg.angular_velocity.x
        self.v_pitch_bottom = msg.angular_velocity.y
        self.v_yaw_bottom = msg.angular_velocity.z
    
    def top_heading_callback(self, msg):
        self.ENU_heading = math.radians( ( -1.0 * msg.heading) + 90.0) 
        
    def publish_clock(self):
        nowTime = self.get_clock().now()
        self.clock_msg.clock.sec = nowTime.seconds_nanoseconds()[0]
        self.clock_msg.clock.nanosec = nowTime.seconds_nanoseconds()[1]
        self.clock_spoofer.publish(self.clock_msg )


    def publish_odometry(self):
        odometry_msg = Odometry()
        odometry_msg.header.stamp = self.get_clock().now().to_msg()
        odometry_msg.header.frame_id = 'utm'
        odometry_msg.child_frame_id = 'gps_top_ant1'
        odometry_msg.pose.pose.position.x = self.true_easting 
        odometry_msg.pose.pose.position.y = self.true_northing  
        odometry_msg.pose.pose.position.z = self.height
        odometry_msg.pose.pose.orientation.x = self.qx 
        odometry_msg.pose.pose.orientation.y = self.qy
        odometry_msg.pose.pose.orientation.z = self.qz
        odometry_msg.pose.pose.orientation.w = self.w
        odometry_msg.twist.twist.linear.x = self.v_x  #velocity in vehicle's frame
        odometry_msg.twist.twist.linear.y = self.v_y
        odometry_msg.twist.twist.linear.z = 0.0 # assume no vertical speed until flying cars are invented 
        odometry_msg.twist.twist.angular.x = self.v_roll  
        odometry_msg.twist.twist.angular.y = self.v_pitch  
        odometry_msg.twist.twist.angular.z = 0.0
        covariance = np.zeros((6, 6))
        # np.fill_diagonal(covariance, 0.001)  
        odometry_msg.pose.covariance = covariance.flatten().tolist()
        self.top_odom_publisher.publish(odometry_msg) #, should be ENU
        
        odometry_msg_bottom = Odometry()
        odometry_msg_bottom.header.stamp = self.get_clock().now().to_msg()
        odometry_msg_bottom.header.frame_id = 'utm'
        odometry_msg_bottom.child_frame_id = 'gps_bottom_ant1'
        odometry_msg_bottom.pose.pose.position.x = self.true_easting_bottom 
        odometry_msg_bottom.pose.pose.position.y = self.true_northing_bottom  
        odometry_msg_bottom.pose.pose.position.z = self.height_bottom
        odometry_msg_bottom.pose.pose.orientation.x = self.qx_bottom 
        odometry_msg_bottom.pose.pose.orientation.y = self.qy_bottom
        odometry_msg_bottom.pose.pose.orientation.z = self.qz_bottom
        odometry_msg_bottom.pose.pose.orientation.w = self.w_bottom
        odometry_msg_bottom.twist.twist.linear.x = self.v_x_bottom   #velocity in vehicle's frame, x is longitudinal, y is lateral
        odometry_msg_bottom.twist.twist.linear.y = self.v_y_bottom  
        odometry_msg_bottom.twist.twist.linear.z = 0.0 # assume no vertical speed 
        odometry_msg_bottom.twist.twist.angular.x = self.v_roll_bottom  
        odometry_msg_bottom.twist.twist.angular.y = self.v_pitch_bottom  
        odometry_msg_bottom.twist.twist.angular.z = 0.0
        covariance = np.zeros((6, 6))
        # np.fill_diagonal(covariance, 0.001)  
        odometry_msg_bottom.pose.covariance = covariance.flatten().tolist()
        self.bottom_odom_publisher.publish(odometry_msg_bottom) #, should be ENU        

    def publish_imu_data(self):
        imu_msg = self.imu_data 
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation_covariance = [0.0] * 9
        imu_raw_msg = imu_msg #no orientation
        imu_raw_msg.header.frame_id = 'gps_top_imu'  #need to do a rotation 90 deg
        self.top_imu_data_raw_publisher.publish(imu_raw_msg)
        imu_msg.header.frame_id = 'gps_top' 
        imu_msg.orientation.x = self.qx #should be orientation in quarternion form, ENU
        imu_msg.orientation.y = self.qy
        imu_msg.orientation.z = self.qz
        imu_msg.orientation.w = self.w
        self.top_imu_data_publisher.publish(imu_msg)
        
        imu_msg_bottom = self.imu_data_bottom 
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation_covariance = [0.0] * 9
        imu_raw_msg_bottom = imu_msg_bottom #no orientation
        imu_raw_msg_bottom.header.frame_id = 'gps_bottom_imu'        
        self.bottom_imu_data_raw_publisher.publish(imu_raw_msg_bottom)
        imu_msg_bottom.header.frame_id = 'gps_bottom' 
        imu_msg_bottom.orientation.x = self.qx_bottom 
        imu_msg_bottom.orientation.y = self.qy_bottom
        imu_msg_bottom.orientation.z = self.qz_bottom
        imu_msg_bottom.orientation.w = self.w_bottom
        self.bottom_imu_data_publisher.publish(imu_msg_bottom)
        
    def publish_ins_twist(self):
        ins_twist_msg = TwistWithCovarianceStamped()
        ins_twist_msg.header.stamp = self.get_clock().now().to_msg()
        ins_twist_msg.header.frame_id = 'gps_top'
        ins_twist_msg.twist.twist.linear.x = self.v_x 
        ins_twist_msg.twist.twist.linear.y = self.v_y 
        ins_twist_msg.twist.twist.linear.z = 0.0 
        ins_twist_msg.twist.twist.angular.x = self.v_roll  
        ins_twist_msg.twist.twist.angular.y = self.v_pitch  
        ins_twist_msg.twist.twist.angular.z = 0.0
        self.top_ins_twist_publisher.publish(ins_twist_msg)
        
        ins_twist_msg_bottom = TwistWithCovarianceStamped()
        ins_twist_msg_bottom.header.stamp = self.get_clock().now().to_msg()
        ins_twist_msg_bottom.header.frame_id = 'gps_top'
        ins_twist_msg_bottom.twist.twist.linear.x = self.v_x_bottom 
        ins_twist_msg_bottom.twist.twist.linear.y = self.v_y_bottom
        ins_twist_msg_bottom.twist.twist.linear.z = 0.0 
        ins_twist_msg_bottom.twist.twist.angular.x = self.v_roll_bottom 
        ins_twist_msg_bottom.twist.twist.angular.y = self.v_pitch_bottom  
        ins_twist_msg_bottom.twist.twist.angular.z = 0.0
        self.bottom_ins_twist_publisher.publish(ins_twist_msg_bottom)
    
    def publish_fix(self):
        fix_msg=NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = 'gps_top_ant1' 
        fix_msg.status.status = 2
        fix_msg.status.service = 3
        fix_msg.latitude = self.latitude
        fix_msg.longitude = self.longitude
        fix_msg.altitude = self.height
        fix_msg.position_covariance = [0.0] * 9
        fix_msg.position_covariance[0] = 0.0  
        fix_msg.position_covariance[4] = 0.0  
        fix_msg.position_covariance[8] = 0.0  
        fix_msg.position_covariance_type = 2
        self.top_fix_publisher.publish(fix_msg)
        
        fix_msg_bottom=NavSatFix()
        fix_msg_bottom.header.stamp = self.get_clock().now().to_msg()
        fix_msg_bottom.header.frame_id = 'gps_bottom_ant1' 
        fix_msg_bottom.status.status = 2
        fix_msg_bottom.status.service = 3
        fix_msg_bottom.latitude = self.latitude_bottom
        fix_msg_bottom.longitude = self.longitude_bottom
        fix_msg_bottom.altitude = self.height_bottom
        fix_msg_bottom.position_covariance = [0.0] * 9
        fix_msg_bottom.position_covariance[0] = 0.0  
        fix_msg_bottom.position_covariance[4] = 0.0  
        fix_msg_bottom.position_covariance[8] = 0.0  
        fix_msg_bottom.position_covariance_type = 2        
        self.bottom_fix_publisher.publish(fix_msg_bottom)


# ============== Helper Functions ================

# https://stackoverflow.com/questions/9186496/determining-utm-zone-to-convert-from-longitude-latitude

    def get_utm_zone(self, longitude): #may not be accurate for certain areas around Norway
        return math.floor(((longitude + 180) % 360) / 6) + 1
    
    def latlon_to_utm(self, latitude, longitude):
        utm_zone = self.get_utm_zone(longitude)
        utm_proj = CRS.from_string(f'EPSG:{32600 + utm_zone}') # Northern Hemisphere is EPSG 326xx
        # utm_proj = CRS.from_string(f'EPSG:{32700 + utm_zone}')  # Southern Hemisphere is EPSG 327xx
        transformer = Transformer.from_crs(CRS.from_epsg(4326), utm_proj, always_xy=True) # Lat/Lon CRS on the WGS84 reference ellipsoid is EPSG 4326 
        # Transform latitude and longitude to UTM coordinates
        utm_easting, utm_northing = transformer.transform(longitude, latitude)

        return utm_easting, utm_northing

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def calculate_local_velocity(self, heading, east_velocity, north_velocity):
        """
        Calculate the longitudinal and lateral velocity in the vehicle's local frame.

        :param heading: The heading of the vehicle in radians (ENU).
        :param global_velocity_x: The velocity of the vehicle in the global +x direction (east).
        :param global_velocity_y: The velocity of the vehicle in the global +y direction (north).
        :return: Tuple containing longitudinal and lateral velocities in the local frame.
        """
        
        cos_heading = math.cos(heading)
        sin_heading = math.sin(heading)
        longitudinal_velocity = cos_heading * east_velocity + sin_heading * north_velocity
        lateral_velocity = -sin_heading * east_velocity + cos_heading * north_velocity
        
        return longitudinal_velocity, lateral_velocity

def main(args=None):
    rclpy.init(args=args)
    node = RepubEkfNode()
    print("republishing to ekf topics...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


