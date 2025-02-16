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
from nav_msgs.msg import Path
from std_msgs.msg import Float32 
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PointStamped, Point, PoseStamped
import os
from ament_index_python.packages import get_package_share_directory
import csv

class GlobalPathPublisher(Node):
    def __init__(self):    
        
        super().__init__('global_path_publisher')
        
        self.publisher_global = self.create_publisher(
            Path, 
            '/global_path', 
            10)
        
        self.global_path = Path()
        self.global_path.header.frame_id = 'map'
        self.positions = [] # positions along raceline
        self.position = PointStamped() 
        

        map_name = self.declare_parameter('map', 'monza').value
        print("Loading map: " + map_name)
        csv_dir = self.declare_parameter('map_dir', 
                os.path.join(get_package_share_directory('planner_emulator'), 'maps')).value
        csv_file_param_name = f"{map_name}.csv_file"
        col_order_param_name = f"{map_name}.col_order"
        csv_file = self.declare_parameter(csv_file_param_name, 'monza.csv').value #default map is monza
        self.col_order = self.declare_parameter(col_order_param_name, [1,2,5,4]).value #default column order
        input_file = os.path.join(csv_dir, csv_file)
        
        if input_file:
            data_x_y_v_k = self.read_specific_columns(input_file)
            self.assign_data_to_path(data_x_y_v_k, "global")
        else:
            self.straight_line()
        self.publisher_global.publish(self.global_path) #initially publish global path

        print("Loading left map: " + map_name + "_left")
        csv_file_left_name, csv_file_ext = os.path.splitext(csv_file)
        csv_file_left = f"{csv_file_left_name}_left{csv_file_ext}"  # Append '_left' to the file name

        input_file_left = os.path.join(csv_dir, csv_file_left)
        
        self.timer = self.create_timer(0.01, self.timer_callback) 
        
    def timer_callback(self):
        self.publisher_global.publish(self.global_path)
        # self.publisher_left.publish(self.left_path)
        

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
    
    def assign_data_to_path(self, data, type):
        for x, y, v, k in data:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            if(type == "global"):
                self.global_path.poses.append(pose)
            self.positions.append([x, y])

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
    

    
        
def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()