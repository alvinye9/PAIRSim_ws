#!/usr/bin/env python3


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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Float32, Float32MultiArray, Bool
import tf2_ros
import tf2_py
from tf2_geometry_msgs import do_transform_point, do_transform_pose_stamped
from ament_index_python.packages import get_package_share_directory
import numpy as np
from scipy.spatial import cKDTree
import csv
import os
import pdb


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_global = self.create_publisher(Path, '/global_path', 10)
        self.publisher_local = self.create_publisher(Path, '/planning/front_path/offset_path', 10)
        self.publisher_vel = self.create_publisher(Float32, '/planning/desired_velocity', 10)
        self.publish_vel_array = self.create_publisher(Float32MultiArray, 'velocities', 10) #future velocities
        self.publish_curvature = self.create_publisher(Float32MultiArray, '/planning/front_path/curvature', 10)
        self.flag_sub = self.create_subscription(Bool, '/is_overtaking', self.flag_callback, 10 )
        self.is_overtaking = False

        self.declare_parameter("velocity_scalar", 1.0) #adjust this to easily scale race trajectory velocity profile
        

        self.global_path = Path()
        self.global_path.header.frame_id = 'map'

        map_name = self.declare_parameter('map', 'monza').value
        print("Loading map: " + map_name)
        csv_dir = self.declare_parameter('map_dir', 
                os.path.join(get_package_share_directory('planner_emulator'), 'maps')).value

        csv_file_param_name = f"{map_name}.csv_file"
        col_order_param_name = f"{map_name}.col_order"

        csv_file = self.declare_parameter(csv_file_param_name, 'monza.csv').value
        self.col_order = self.declare_parameter(col_order_param_name, [1,2,5,4]).value

        input_file = os.path.join(csv_dir, csv_file)


        # Positions will be used to build KDTree
        self.positions = []
        self.velocities = []
        self.curvatures = []
        self.vel_scalar = self.get_parameter("velocity_scalar").value
        self.prev_vel = 0

        self.num_laps = 0
        self.crossing_line = False
        self.lap_start_time = self.get_clock().now()

        if input_file:
            data_x_y_v_k = self.read_specific_columns(input_file)
            self.assign_data_to_path(data_x_y_v_k)
        else:
            self.straight_line()
        
        #self.straight_line()

        self.publisher_global.publish(self.global_path) 
    
        # KDTree allows for quick lookup of points
        self.kdtree = cKDTree(self.positions)

        # Setup storage for TF buffer and define listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transform = None

        timer_period = 0.01  # seconds
        # Timer to call our publish function
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def flag_callback(self, msg):
        self.is_overtaking = msg.data
        
    def straight_line(self):
        for i in range(10000):
            x = i * 0.1
            y = 0.0
            pose = PoseStamped()
            pose.header.frame_id = 'map' #tells us which frame this path is in
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.global_path.poses.append(pose)
            self.positions.append([x, y])
    
    def assign_data_to_path(self, data):
        #print("assigning csv data to path msg")
        #print(data)

        #WORK ON ADDING ORIENTATION?
        for x, y, v, k in data:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.global_path.poses.append(pose)
            self.positions.append([x, y])
            self.velocities.append(v)
            self.curvatures.append(k)


    def read_specific_columns(self, input_file):
        with open(input_file, 'r') as csvfile:
            sample = csvfile.read(1024)
            csvfile.seek(0) 
            dialect = csv.Sniffer().sniff(sample)

            reader = csv.reader(csvfile, dialect)

            header = next(reader)  # Read the header row
            x_i = self.col_order[0]
            y_i = self.col_order[1]
            v_i = self.col_order[2]
            k_i = self.col_order[3]
            values = [(float(row[x_i]), float(row[y_i]), float(row[v_i]), float(row[k_i])) for row in reader]
            #print(values)
        return values #return all the values within these columns
    

    def timer_callback(self):
        # self.get_logger().info(f"Overtaking: {self.is_overtaking}")
        # Try to get transform vehicle->map, return if fails
        self.vel_scalar = float(self.get_parameter("velocity_scalar").value);

        try:
            self.transform = self.tf_buffer.lookup_transform("map", "vehicle", rclpy.time.Time()) 
        except Exception as e:
            print(e)
            return

        orig = PointStamped()
        orig.point.x, orig.point.y = 0.0, 0.0

        # Transforming (0, 0) in car frame to global frame gives global car coordinates
        # there has to be a better way to do this
        car_loc = do_transform_point(orig, self.transform)
        x, y = car_loc.point.x, car_loc.point.y

        # Find closest point on path to global point
        _, idx = self.kdtree.query([x, y])
    
        global_path = Path()
        global_path.header.frame_id = 'map'

        local_path = Path()
        local_path.header.frame_id = 'vehicle'

        if (idx + 400) % len(self.positions) < idx and (idx + 350) % len(self.positions) > idx:
            self.crossing_line = True
        else:
            if self.crossing_line:
                self.crossing_line = False
                self.num_laps += 1
                if self.num_laps > 0:
                    print(f"Lap: {self.num_laps}, Time: {(self.get_clock().now() - self.lap_start_time).nanoseconds/1e9}, velocity scalar: {self.vel_scalar}")
                #self.vel_scalar += 0.1
                self.lap_start_time = self.get_clock().now()

        velocities = Float32MultiArray()
        curvatures = Float32MultiArray()

        transform = self.tf_buffer.lookup_transform('vehicle', 'map', rclpy.time.Time())

        for i in range(idx, idx+100):
            i = i % len(self.global_path.poses)
            pose = do_transform_pose_stamped(self.global_path.poses[i], transform)
            local_path.poses.append(pose)
            velocities.data.append(self.velocities[i]*self.vel_scalar)
            curvatures.data.append(self.curvatures[i])

            pose2 = self.global_path.poses[i]
            global_path.poses.append(pose2)


        vel_msg = Float32()
        vel_msg.data = self.velocities[idx]*self.vel_scalar

        self.publisher_vel.publish(vel_msg)
        
        if self.is_overtaking is False:
            self.publisher_local.publish(local_path)          

        #start = self.get_clock().now()
        # self.publisher_global.publish(self.global_path) #now handled by global_path_publisher
        self.publish_vel_array.publish(velocities)
        self.publish_curvature.publish(curvatures)

        #print(self.get_clock().now()-start)



def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)

    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

