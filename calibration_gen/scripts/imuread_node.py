#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import serial.tools.list_ports
from sensor_msgs.msg import Imu
import numpy as np
from ament_index_python import get_package_share_directory
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

        self.gx_offset = 0.050949999999999364
        self.gy_offset = -0.0416299999999994
        self.gz_offset = -0.0499899999999993

        self.ax_offset = 0.045919999999999614
        self.ay_offset = 0.2553499999999982
        self.az_offset = 9.764280000000126

        self.isCalibrated = False


        # [INFO] [1707379980.026784490] [imu_calibration_node]: Calibration complete: 
        # GX offset: 2.0350999999999995, 
        # GY offset: 0.051550000000000006, 
        # GZ offset: -1.6954200000000001

        calibration_gen_path = get_package_share_directory('calibration_gen')
        self.path = os.path.join(calibration_gen_path, 'config', 'sensor_calibration.yaml')


    def imu_data_callback(self, msg):

        with open(self.path, 'r') as file:
            value = yaml.safe_load(file)

        # if self.isCalibrated == True:
        imu_msg_cal = Imu()
        imu_msg_cal.header.stamp = self.get_clock().now().to_msg()
        imu_msg_cal.header.frame_id = "imu_link"  # Adjust as needed
        # Gyroscope data in rad/s
        imu_msg_cal.angular_velocity.x = msg.angular_velocity.x - value['offset gyro'][0]
        imu_msg_cal.angular_velocity.y = msg.angular_velocity.y - value['offset gyro'][1]
        imu_msg_cal.angular_velocity.z = msg.angular_velocity.z - value['offset gyro'][2]
        cov_gyro_np = np.array(value['cov gyro'])
        imu_msg_cal.angular_velocity_covariance = cov_gyro_np.flatten()
        
        # Accelerometer data in m/s^2
        imu_msg_cal.linear_acceleration.x = msg.linear_acceleration.x - value['offset acc'][0]
        imu_msg_cal.linear_acceleration.y = msg.linear_acceleration.y - value['offset acc'][1]
        imu_msg_cal.linear_acceleration.z = msg.linear_acceleration.z - value['offset acc'][2]
        cov_acc_np = np.array(value['cov acc'])
        imu_msg_cal.linear_acceleration_covariance = cov_acc_np.flatten()

        print(imu_msg_cal)

        self.publisher_imu_cal.publish(imu_msg_cal)


def main(args=None):
    rclpy.init(args=args)
    imu_serial_reader = IMUSerialReader()
    rclpy.spin(imu_serial_reader)
    imu_serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
