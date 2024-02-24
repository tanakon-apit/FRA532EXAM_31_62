#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np
from ament_index_python import get_package_share_directory
from tf_transformations import quaternion_from_euler
import os
import sys, yaml

class IMUSerialReader(Node):
    def __init__(self):
        super().__init__('imu_serial_reader')
        # self.timer_period = 0.02  # seconds
        # self.timer = self.create_timer(self.timer_period, self.read_and_publish)
        self.create_subscription(Imu, '/imu_raw', self.imu_data_callback, 10)
        self.publisher_imu_cal = self.create_publisher(Imu, 'imu', 10)
        # self.cli = self.create_client(Bool, 'Calibration')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')                              

        self.isCalibrated = False
        
        calibration_gen_path = get_package_share_directory('calibration_gen')
        self.path = os.path.join('/home/tuchapong1234/FRA532EXAM_WS', 'config', 'sensor_calibration.yaml')
        with open(self.path, 'r') as file:
            self.value = yaml.safe_load(file)

        self.imu_msg_cal = Imu()
        cov_gyro_np = np.array(self.value['cov gyro'])
        # self.imu_msg_cal .angular_velocity_covariance = cov_gyro_np.flatten()
        cov_acc_np = np.array(self.value['cov acc'])
        self.imu_msg_cal .linear_acceleration_covariance = cov_acc_np.flatten()

        self.quat = quaternion_from_euler(0.0, 0.0, 0.0)
        self.lasttimestamp = self.get_clock().now()

        self.angle = 0.0
    
    def imu_data_callback(self, msg : Imu):
        
        currenttimestamp = self.get_clock().now()
        dt = (currenttimestamp - self.lasttimestamp).to_msg().nanosec * 1.0e-9
        self.lasttimestamp = currenttimestamp
        # if self.isCalibrated == True:
        self.imu_msg_cal .header.stamp = self.get_clock().now().to_msg()
        self.imu_msg_cal .header.frame_id = "imu"  # Adjust as needed
        # Gyroscope data in rad/s
        self.imu_msg_cal .angular_velocity.x = msg.angular_velocity.x - self.value['offset gyro'][0]
        self.imu_msg_cal .angular_velocity.y = msg.angular_velocity.y - self.value['offset gyro'][1]
        self.imu_msg_cal .angular_velocity.z = msg.angular_velocity.z - self.value['offset gyro'][2]
        
        # Accelerometer data in m/s^2
        self.imu_msg_cal .linear_acceleration.x = msg.linear_acceleration.x - self.value['offset acc'][0]
        self.imu_msg_cal .linear_acceleration.y = msg.linear_acceleration.y - self.value['offset acc'][1]
        self.imu_msg_cal .linear_acceleration.z = msg.linear_acceleration.z - self.value['offset acc'][2]

        self.angle += self.imu_msg_cal.angular_velocity.z * dt
        self.quat = quaternion_from_euler(0.0, 0.0, self.angle)
        self.imu_msg_cal.orientation = Quaternion(x=self.quat[0], y=self.quat[1], z=self.quat[2], w=self.quat[3])

        print(self.imu_msg_cal )
        self.publisher_imu_cal.publish(self.imu_msg_cal )


def main(args=None):
    rclpy.init(args=args)
    imu_serial_reader = IMUSerialReader()
    rclpy.spin(imu_serial_reader)
    imu_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
