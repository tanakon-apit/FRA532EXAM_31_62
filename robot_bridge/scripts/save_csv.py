#!/usr/bin/python3

from robot_bridge.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import csv
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class CSVWritterNode(Node):
    def __init__(self):
        super().__init__('csv_writer_node')
        self.create_subscription(Float64MultiArray, "/wheel_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_filter_callback, 10)
        self.create_subscription(Odometry, "/cmd", self.cmd_callback, 10)
        self.create_timer(0.033, self.timer_callback)
        self.odom_x = None
        self.odom_y = None
        self.odom_filter_x = None
        self.odom_filter_y = None
        self.cmd_x = None
        self.cmd_y = None

        self.cmd_vx = 0
        self.cmd_w = 0

        # name of csv file
        filename = "university_records.csv"
    
        self.path = os.path.join('/home/tuchapong1234/FRA532EXAM_WS', filename)

        # field names
        fields = ['cmd_x', 'cmd_y', 'odom_x', 'odom_y', 'odom_filter_x', 'odom_filter_y']

        # writing to csv file
        with open(self.path, 'w') as csvfile:
            # creating a csv writer object
            csvwriter = csv.writer(csvfile)
            # writing the fields
            csvwriter.writerow(fields)

    def timer_callback(self):
            
        if self.cmd_vx == 0.0 and self.cmd_w == 0.0:
            print("Stop Record: Robot Stop")
        else:
            print("Wheel Vel: ", self.cmd_vx, self.cmd_w)
            print("Record: ", self.cmd_x, self.cmd_y, self.odom_x, self.odom_y, self.odom_filter_x, self.odom_filter_y)
            # Check if all data is available
            if self.odom_x is not None and self.odom_filter_x is not None and self.odom_y is not None and self.odom_filter_y is not None and self.cmd_y is not None and self.cmd_x is not None:
                # Write data to CSV
                with open(self.path, 'a') as csvfile:  # Use 'a' mode to append
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerow([self.cmd_x, self.cmd_y, self.odom_x, self.odom_y, self.odom_filter_x, self.odom_filter_y])
                # Reset data
                self.odom_x = None
                self.odom_y = None
                self.odom_filter_x = None
                self.odom_filter_y = None
                self.cmd_x = None
                self.cmd_y = None

    def cmd_vel_callback(self, msg:Float64MultiArray):
        self.cmd_vx = msg.data[0]
        self.cmd_w = msg.data[1]


    def odom_callback(self, msg:Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def odom_filter_callback(self, msg:Odometry):
        self.odom_filter_x = msg.pose.pose.position.x
        self.odom_filter_y = msg.pose.pose.position.y

    def cmd_callback(self, msg:Odometry):
        self.cmd_x = msg.pose.pose.position.x
        self.cmd_y = msg.pose.pose.position.y

def main(args=None):
    rclpy.init(args=args)
    node = CSVWritterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
