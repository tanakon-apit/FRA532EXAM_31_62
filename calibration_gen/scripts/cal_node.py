#!/usr/bin/python3


import os
from signal import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python import get_package_share_directory
from sensor_msgs.msg import Imu
import sys, yaml
import numpy as np
from std_msgs.msg import Bool
class CalibrationSensor(Node):
    def __init__(self):
        super().__init__('CalibrationSensor')
        # self.create_subscription(Float64MultiArray, '/sensor_data', self.sensor_data_callback, 10)
        self.create_subscription(Imu, '/imu_raw', self.imu_data_callback, 10)
        # self.srv = self.create_service(Bool, 'Calibration', self.status_callback)

        self.collected_data = []

        self.collected_gyro_data = []
        self.collected_acc_data = []

        self.n = 0
        self.num = 2000

        self.isCalibrated = False

        calibration_gen_path = get_package_share_directory('calibration_gen')
        print(calibration_gen_path)
        self.path = os.path.join('/home/tuchapong1234/FRA532EXAM_WS/', 'config', 'sensor_calibration.yaml')

    def timer_callback(self):
        pass

    # def status_callback(self, request, response):
    #     if self.isCalibrated == True:
    #         response.result = True
    #     else:
    #         response.result = False
    #     return response

    def save_calibration(self, mean, cov, name):
        
        with open(self.path, 'r') as file:
            value = yaml.safe_load(file)

        mean_list = mean.tolist()
        cov_list = cov.tolist()

        print(mean_list)
        print(cov_list)

        print(value)
        
        value['offset {}'.format(name)] = mean_list
        value['cov {}'.format(name)] = cov_list

        with open(self.path, 'w') as file:
            yaml.dump(value, file)
        
        print("Calibrated", value)

    def imu_data_callback(self, msg):
        if self.n < self.num and self.isCalibrated == False:
            self.n = self.n + 1
            self.collected_acc_data.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            self.collected_gyro_data.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            print("collect data: " + str(self.n)) 
        else:
            if self.isCalibrated == False:
                data_acc_array = np.array(self.collected_acc_data)
                offset_acc = np.mean(data_acc_array, 0)
                cov_acc = np.cov(data_acc_array.T)

                self.save_calibration(offset_acc, cov_acc, 'acc')

                data_gyro_array = np.array(self.collected_gyro_data)
                offset_gyro = np.mean(data_gyro_array, 0)
                cov_gyro = np.cov(data_gyro_array.T)

                self.save_calibration(offset_gyro, cov_gyro, 'gyro')
                
                print("=====================")
                print(offset_acc)
                print(cov_acc)
                print("=====================")
                print(offset_gyro)
                print(cov_gyro)
                self.isCalibrated = True
                exit()
            
    
def main(args=None):
    rclpy.init(args=args)
    print("A")
    node = CalibrationSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
