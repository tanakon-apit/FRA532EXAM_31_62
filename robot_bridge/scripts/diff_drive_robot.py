#!/usr/bin/python3

from lab1.mobile_kinematic import Diff_Drive_Kinematic
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np

class DiffDriveRobot(Node):
    def __init__(self):
        super().__init__('diff_drive_robot')
        
        self.create_subscription(Float64MultiArray, "/wheel_vel", self.qd_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_tf_br = TransformBroadcaster(self)
        self.create_timer(0.033, self.timer_callback)

        self.Kine = Diff_Drive_Kinematic(r=0.03375, b=0.1625)
        self.qd = np.zeros(2)
        self.pose = np.zeros(3)
        self.twist = np.zeros(2)
        self.quat = quaternion_from_euler(0.0, 0.0, self.pose[2])
        self.lasttimestamp = self.get_clock().now()

        self.twist_cov = (np.eye(6) * 1.0e-3).flatten()
        self.pose_cov = (np.eye(6) * 1.0e3).flatten()

        #Test Wheel odom
        # self.create_subscription(Twist, "/command", self.cmd_callback, 10)
        # self.vel_cont = self.create_publisher(Float64MultiArray, "/qd", 10)
    
    def timer_callback(self):
        # calculate dt
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp
        # update odometry
        self.pose = self.Kine.get_pose(dt=dt, qd=self.qd)
        self.twist = self.Kine.get_twist(qd=self.qd)
        # calculate quaternion angle
        self.quat = quaternion_from_euler(0.0, 0.0, self.pose[2])
        # publish odometry and transformation
        self.pub_odometry()
        # self.pub_transformation()

    # def cmd_callback(self, msg):
    #     # subscribe cmd_vel and transform to wheel speed
    #     qd = Float64MultiArray()
    #     qd.data = self.Kine.get_wheelspeed(
    #         [msg.linear.x,
    #          msg.angular.z]).tolist()
        
    #     #publish qd
    #     self.vel_cont.publish(qd)
        
    def qd_callback(self, msg):
        self.qd = msg.data

    def pub_transformation(self):
        tf_stamp = TransformStamped()
        tf_stamp.header.stamp = self.get_clock().now().to_msg()
        tf_stamp.header.frame_id = "odom"
        tf_stamp.child_frame_id = "base_link"
        tf_stamp.transform.translation.x = self.pose[0]
        tf_stamp.transform.translation.y = self.pose[1]
        tf_stamp.transform.rotation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        self.pub_tf_br.sendTransform(tf_stamp)

    def pub_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.covariance = self.pose_cov
        odom.pose.pose.orientation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        odom.twist.twist.linear.x = self.twist[0]
        odom.twist.twist.angular.z = self.twist[1]
        odom.twist.covariance = self.twist_cov
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()