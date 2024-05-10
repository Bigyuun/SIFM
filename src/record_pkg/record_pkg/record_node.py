import sys, os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
import rosbag2_py._storage
from std_srvs.srv import SetBool
from ros2bag.api import rosbag2_py
import rosbag2_py
from rclpy.serialization import serialize_message

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from custom_interfaces.msg import LoadcellState
from custom_interfaces.msg import MotorCommand
from custom_interfaces.msg import MotorState
from custom_interfaces.srv import MoveMotorDirect
from custom_interfaces.srv import MoveToolAngle

import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from datetime import datetime

import csv
import subprocess   # CLI

class RecordNode(Node):
    def __init__(self):
        """
        ROS2 bag profile set
        """
        super().__init__('record_node')
        self.create_service(SetBool, '/data/record', self.record_callback)
        self.is_recording = False
        self.data_count = 0

        # self.rosbag_writer = rosbag2_py.SequentialWriter()
        # storage_options = rosbag2_py._storage.StorageOptions(
        #     uri='SIFM_bag',
        #     storage_id='sqlite3')
        # converter_options = rosbag2_py._storage.ConverterOptions('', '')
        # self.rosbag_writer.open(storage_options, converter_options)

        # topic_info_fts_data = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='fts_data_raw',
        #     type='geometry_msgs/msg/WrenchStamped',
        #     serialization_format='cdr')
        # topic_info_loadcell = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='loadcell_state',
        #     type='custom_interfaces/msg/LoadcellState',
        #     serialization_format='cdr')
        # topic_info_motor = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='motor_state',
        #     type='custom_interfaces/msg/MotorState',
        #     serialization_format='cdr')
        # topic_info_image = rosbag2_py._storage.TopicMetadata(
        #     # id=0,
        #     name='camera/color/image_rect_raw',
        #     type='sensor_msgs/msg/Image',
        #     serialization_format='cdr')
        
        # self.rosbag_writer.create_topic(topic_info_fts_data)
        # self.rosbag_writer.create_topic(topic_info_loadcell)
        # self.rosbag_writer.create_topic(topic_info_motor)
        # self.rosbag_writer.create_topic(topic_info_image)

        # ROS2 topic subscriber
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.motor_command_publisher_ = self.create_publisher(MotorCommand, 'motor_command', QOS_RKL10V)
        
        self.fts_data_flag = False
        self.fts_data = WrenchStamped()
        self.fts_subscriber = self.create_subscription(
            WrenchStamped,
            'fts_data_raw',
            self.read_fts_data,
            QOS_RKL10V
        )
        self.get_logger().info('fts_data subscriber is created.')

        self.loadcell_data_flag = False
        self.loadcell_data = LoadcellState()
        self.lc_subscriber = self.create_subscription(
            LoadcellState,
            'loadcell_state',
            self.read_loadcell_data,
            QOS_RKL10V
        )
        self.get_logger().info('loadcell_data subscriber is created.')

        self.motor_state_flag = False
        self.motor_state = MotorState()
        self.motor_state_subscriber = self.create_subscription(
            MotorState,
            'motor_state',
            self.read_motor_state,
            QOS_RKL10V
        )
        self.get_logger().info('motor_state subscriber is created.')


        # self.realsense_subscriber = RealSenseSubscriber()
        # color rectified image. RGB format
        self.image_flag = False
        self.br_rgb = CvBridge()
        self.color_image_rect_raw_subscriber = self.create_subscription(
            Image,
            "camera/color/image_raw",
            # "camera/color/image_rect_raw",
            self.color_image_rect_raw_callback,
            QOS_RKL10V)
        self.get_logger().info('realsense-camera subscriber is created.')


        ### ================================================================
        ### file managers
        ### ================================================================
        # hw_definition.hpp 파일의 경로 설정
        print(os.getcwd())
        hw_definition_hpp_path = './src/kinematics_control_pkg/include/kinematics_control_pkg/hw_definition.hpp'
        # 파싱하여 상수 값을 읽어옴
        constants = self.parse_hw_definition_hpp(hw_definition_hpp_path)
        # 상수 값 출력
        for key, value in constants.items():
            # self.get_logger().info(f'{key}: {value} ({type(key)}/{type(value)})')
            if key == 'NUM_OF_MOTORS':
                self.numofmotors = int(value)
            if key == 'OP_MODE':
                if value == '0x08':
                    self.get_logger().info(f'OP_MODE: CSP')
                elif value == '0x09':
                    self.get_logger().info(f'OP_MODE: CSV')
                self.opmode = value

        self.directory_path = None
        self.directory_path_image = None
        self.directory_path_csv = None

    
    ### ================================================================
    ### Functions
    ### ================================================================
        """ROS2 Functions
        - callback functions of subscribers, service_server
        """ 
    def record_callback(self, request, response):
        self.is_recording = request.data

        try:
            if self.is_recording:
                # Start recording
                self.get_logger().info('Start recording')
                # os.chdir(self.directory_path)
                self.create_directory()
                self.create_csv()
                self.bag_process = subprocess.Popen(['ros2', 'bag', 'record', '-a'], cwd=self.directory_path)
                response.success = True
                response.message = 'Start Recording.'
            elif self.is_recording == False:
                # Subscribe to topics you want to record
                self.get_logger().info('Stop recording')
                self.bag_process.terminate()
                self.csv_file.close()
                response.success = True
                response.message = 'Stop Recording.'
                self.data_count = 0
        except Exception as e:
            print(f'Exception Error as {e}')
            response.success = False
            response.message = 'Error is up for recording.'

        return response

    # def stop_record_callback(self, request, response):
    #     if self.recorder:
    #         # Stop recording
    #         self.get_logger().info('Stop recording')
    #         self.recorder.stop()
    #         self.recorder = None
    #     else:
    #         self.get_logger().info('Not recording')
    #     return response


    
    def color_image_rect_raw_callback(self, data):
        """_summary_

        Args:
            data (_type_): _description_
        """
        # self.get_logger().info("Receiving RGB frame")
        self.image_flag = True
        self.capture_time = data.header.stamp
        self.current_frame = self.br_rgb.imgmsg_to_cv2(data, 'bgr8')

        if self.is_recording:
            self.data_count += 1
            cv2.imwrite(self.directory_path_image + '/' + str(self.data_count) +'.png', self.current_frame)
            self.update_csv()

        cv2.imshow("[Record Node] rgb", self.current_frame)
        cv2.waitKey(1)
        # if self.is_recording:
        #     self.rosbag_writer.write(
        #     'camera/color/image_rect_raw',
        #     serialize_message(data),
        #     self.get_clock().now().nanoseconds
        # )

        # return

    def read_fts_data(self, msg):
        self.fts_data_flag = True
        self.fts_data = msg

    def read_loadcell_data(self, msg):
        self.loadcell_data_flag = True
        self.loadcell_data = msg

    def read_motor_state(self, msg):
        self.motor_state_flag = True
        self.motor_state = msg

    def create_directory(self):
        c_time = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        self.directory_path = os.path.join('./record', c_time)
        if not os.path.exists(self.directory_path):
            os.makedirs(self.directory_path)
            print(f'Directory is created => {self.directory_path}')
        self.directory_path_image = os.path.join(self.directory_path, 'images')
        if not os.path.exists(self.directory_path_image):
            os.makedirs(self.directory_path_image)
        self.directory_path_csv = self.directory_path

    def create_csv(self):
        self.csv_file_name = os.path.join(self.directory_path_csv, 'data.csv')
        print(f'CSV is created => name : {self.csv_file_name}')
        self.csv_headers = {}
        self.csv_headers['timestamp'] = []
        self.csv_headers['image file'] = []
        for i in range(self.numofmotors):
            self.csv_headers[f'motor #{i}'] = []
        for i in range(self.numofmotors):
            self.csv_headers[f'loadcell #{i}'] = []
        self.csv_headers['fx'] = []
        self.csv_headers['fy'] = []
        self.csv_headers['fz'] = []
        self.csv_headers['tx'] = []
        self.csv_headers['ty'] = []
        self.csv_headers['tz'] = []
        # print(f"{self.csv_headers}")
        self.csv_file = open(self.csv_file_name, mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.csv_headers)

    def update_csv(self):
        # if not self.image_flag and not self.fts_data_flag and not self.motor_state_flag and not self.loadcell_data_flag:
        #     self.get_logger().warning(f'All data are not subscribed')
        #     return
        
        timestamp = str(self.capture_time.sec) + str(self.capture_time.nanosec)
        image_file = str(self.data_count) +'.png'
        actual_position = self.motor_state.actual_position
        loadcell_stress = self.loadcell_data.stress
        forcexyz = self.fts_data.wrench.force
        torquexyz = self.fts_data.wrench.torque
        
        # self.csv_writer.write(f"{timestamp}, {image_file}")
        # self.csv_writer.write(",".join(str(value) for value in actual_position))
        # self.csv_writer.write(",")
        # self.csv_writer.write(",".join(str(value) for value in loadcell_stress))
        # self.csv_writer.write(",")
        # self.csv_writer.write(",".join(str(value) for value in forcexyz))
        # self.csv_writer.write(",")
        # self.csv_writer.write(",".join(str(value) for value in torquexyz))
        # self.csv_writer.write("\n")
        # self.csv_writer.writerow([timestamp, image_file]
        #                          + [",".join(map(str, actual_position))]
        #                          + [",".join(map(str, loadcell_stress))]
        #                          + [","+str(forcexyz.x)]
        #                          + [","+str(forcexyz.y)]
        #                          + [","+str(forcexyz.z)]
        #                          + [","+str(torquexyz.z)]
        #                          + [","+str(torquexyz.y)]
        #                          + [","+str(torquexyz.z)])
        
        self.csv_writer.writerow([timestamp, image_file]
                                 + [str(value) for value in actual_position]
                                 + [str(value) for value in loadcell_stress]
                                 + [str(forcexyz.x)]
                                 + [str(forcexyz.y)]
                                 + [str(forcexyz.z)]
                                 + [str(torquexyz.x)]
                                 + [str(torquexyz.y)]
                                 + [str(torquexyz.z)])
        
        self.csv_file.flush()    
        pass
    
    def parse_hw_definition_hpp(self, file_path):
        """
        Parse the given HPP file and extract global variables and their values.
        Ignore lines starting with "//" and lines containing only whitespace.

        Args:
            file_path (str): The path to the HPP file.

        Returns:
            dict: A dictionary containing global variables and their values.
        """
        constants = {}
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if line.startswith('#define'):
                    parts = line.split()
                    if len(parts) >= 3:
                        key = parts[1]
                        value = parts[2]
                        constants[key] = value
        return constants
    
def main(args=None):
    rclpy.init(args=args)
    record_node = RecordNode()
    rclpy.spin(record_node)
    record_node.destroy_node()
    print('record node is destroyed')
    rclpy.shutdown()
    print('rclpy shutdonw')

if __name__ == '__main__':
    main()
