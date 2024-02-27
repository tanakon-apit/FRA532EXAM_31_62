#!/usr/bin/python3

from lab1.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import csv
import os
from nav_msgs.msg import Odometry


class CSVWritterNode(Node):
    def __init__(self):
        super().__init__('csv_writer_node')
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_filter_callback, 10)
        self.create_subscription(Odometry, "/cmd", self.cmd_callback, 10)
        self.create_timer(0.033, self.timer_callback)
        self.odom = None
        self.odom_filter = None
        self.cmd = None

        # name of csv file
        filename = "university_records.csv"
    
        self.path = os.path.join('/home/tuchapong1234/FRA532EXAM_WS', filename)

        # field names
        fields = ['cmd', 'odom', 'odom_filter']

    def timer_callback(self):
        # Check if all data is available
        # if self.odom is not None and self.odom_filter is not None and self.cmd is not None:
        if self.odom is not None:
            # Write data to CSV
            with open(self.path, 'a') as csvfile:  # Use 'a' mode to append
                csvwriter = csv.writer(csvfile)
                csvwriter.writerow([self.cmd, self.odom, self.odom_filter])
            # Reset data
            self.odom = None
            self.odom_filter = None
            self.cmd = None


    def odom_callback(self, msg:Odometry):
        self.odom = msg.pose.pose.position.x

    def odom_filter_callback(self, msg:Odometry):
        self.odom_filter = msg.pose.pose.position

    def cmd_callback(self, msg:Odometry):
        self.cmd = msg.pose.pose.position


        # # data rows of csv file
        # rows = [['Nikhil', 'COE', '2', '9.0'],
        #         ['Sanchit', 'COE', '2', '9.1'],
        #         ['Aditya', 'IT', '2', '9.3'],
        #         ['Sagar', 'SE', '1', '9.5'],
        #         ['Prateek', 'MCE', '3', '7.8'],
        #         ['Sahil', 'EP', '2', '9.1']]
        

        
        # # writing to csv file
        # with open(self.path, 'w') as csvfile:
        #     # creating a csv writer object
        #     csvwriter = csv.writer(csvfile)
        #     # writing the fields
        #     csvwriter.writerow(fields)
        #     # writing the data rows
        #     csvwriter.writerows(rows)



def main(args=None):
    rclpy.init(args=args)
    node = CSVWritterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
