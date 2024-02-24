#!/usr/bin/python3

from lab1.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
from tf_transformations import quaternion_from_euler

class CommandOdomNode(Node):
    def __init__(self):
        super().__init__('Command_Odom')
        self.create_subscription(Twist, "/command", self.cmd_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_pub = self.create_publisher(Odometry, "cmd", 10)
        # self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_tf_br = TransformBroadcaster(self)

        self.time_step = 0.033
        self.create_timer(self.time_step, self.timer_callback)

        self.cmd_vel = [0.0, 0.0]
        self.cmd_pos = [0.0, 0.0, 0.0]
        self.quat = quaternion_from_euler(0.0, 0.0, self.cmd_pos[2])
        self.lasttimestamp = self.get_clock().now()
    
    def cmd_callback(self, msg):
        self.cmd_vel = [msg.linear.x, msg.angular.z]
        self.cmd_vel_pub.publish(msg)

    def integrate(self, dt, cmd_vel, pos):
        vx = cmd_vel[0] * math.cos(cmd_vel[1])
        vy = cmd_vel[0] * math.sin(cmd_vel[1])

        pos[0] = pos[0] + vx*dt
        pos[1] = pos[1] + vy*dt
        pos[2] = pos[2] + cmd_vel[1]*dt

        return pos

    def timer_callback(self):
        # calculate dt
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp
        # Integrate vel to pose
        self.cmd_pos = self.integrate(dt, self.cmd_vel, self.cmd_pos)
        # calculate quaternion angle
        self.quat = quaternion_from_euler(0.0, 0.0, self.cmd_pos[2])
        # publish odometry and transformation
        # self.pub_odometry()
        self.pub_transformation()

    def pub_transformation(self):
        tf_stamp = TransformStamped()
        tf_stamp.header.stamp = self.get_clock().now().to_msg()
        tf_stamp.header.frame_id = "odom"
        tf_stamp.child_frame_id = "cmd"
        tf_stamp.transform.translation.x = self.cmd_pos[0]
        tf_stamp.transform.translation.y = self.cmd_pos[1]
        tf_stamp.transform.rotation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        self.pub_tf_br.sendTransform(tf_stamp)

    def pub_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "cmd"
        odom.pose.pose.position.x = self.cmd_pos[0]
        odom.pose.pose.position.y = self.cmd_pos[1]
        odom.pose.pose.orientation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])
        odom.twist.twist.linear.x = self.cmd_vel[0]
        odom.twist.twist.angular.z = self.cmd_vel[1]
        # self.pub_odom.publish(odom)
    

def main(args=None):
    rclpy.init(args=args)
    node = CommandOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
