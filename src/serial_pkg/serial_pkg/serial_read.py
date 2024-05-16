import serial
import time
import sys, os
import numpy as np
import threading

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Header
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import SetBool
from custom_interfaces.msg import LoadcellState
from custom_interfaces.msg import DataFilterSetting

usb_device_path = "/dev/ttyACM0"  # USB 장치의 경로에 맞게 변경하세요

# USB 장치에 대한 권한 변경 명령어
command = f"sudo chmod 666 {usb_device_path}"

class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        # self.declare_parameter('')
        # self.add_on_set_parameters_callback(self.update_parameter)

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.fts_publisher = self.create_publisher(
            WrenchStamped,
            'fts_data_raw',
            QOS_RKL10V
        )
        
        self.loadcell_publisher = self.create_publisher(
            LoadcellState,
            'loadcell_state',
            QOS_RKL10V
        )

        self.data_filter_setting_subscriber = self.create_subscription(
            DataFilterSetting,
            'data_filter_setting',
            self.data_filter_setting_callback,
            QOS_RKL10V
        )
        # self.LPF_state = Bool()
        # self.LPF_state.data = False
        # self.get_logger().info(f'LPF_state: {self.LPF_state.data}')
        # self.LPF_state_subscriber = self.create_subscription(
        #     Bool,
        #     'LPF_state',
        #     self.LPF_state_callback,
        #     QOS_RKL10V
        # )
        # self.LPF_state_subscriber
        # self.get_logger().info(f'LPF_state_subscriber: {self.LPF_state_subscriber}')

        # self.MAF_state = Bool()
        # self.MAF_state.data = False
        # self.get_logger().info(f'MAF_state: {self.MAF_state.data}')
        # self.MAF_state_subscriber = self.create_subscription(
        #     Bool,
        #     'MAF_state',
        #     self.MAF_state_callback,
        #     QOS_RKL10V
        # )
        # self.MAF_state_subscriber
        # self.get_logger().info(f'MAF_state_subscriber: {self.MAF_state_subscriber}')

        # self.create_service(SetBool, '/serial_data/set_zero', self.set_zero_callback)

        self.serial_port = '/dev/ttyACM0'  # 사용하는 시리얼 포트(COM 포트)를 지정하세요.
        self.baudrate = 115200  # 아두이노와 통신하는 속도
        self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
        self.get_logger().info(f'Serial status: {self.ser}')

        # self.force3d = [0 for i in range(3)]   # empty list [0, 0, 0]
        # self.torque3d = [0 for i in range(3)]   # empty list [0, 0, 0]
        # self.loadcell_weight = [0 for i in range(2)]
        
        self.force3d = np.zeros((1,3))   # empty list [0, 0, 0]
        self.torque3d = np.zeros((1,3))   # empty list [0, 0, 0]
        self.loadcell_weight = np.zeros((1,2))
        # self.get_logger().info(f'shape={self.force3d.shape}')

        self.offset_force3d = np.zeros((1,3))
        self.offset_torque3d = np.zeros((1,3))
        self.offset_loadcell_weight = np.zeros((1,2))

        self.data_filter_setting = DataFilterSetting()
        self.filter_weight_sensitivity = 0.3
        self.size_maf = 5   # default size of moving average filter
        
        self.buffer_count = 0
        self.force3d_buffer = np.zeros((self.size_maf,3))
        self.torque3d_buffer = np.zeros((self.size_maf,3))
        self.loadcell_weight_buffer = np.zeros((self.size_maf,2))

        self.get_logger().info('serial node is up.')
        while True:
            suc = self.set_zero(100)
            self.get_logger().info(f'SET ZERO: {suc}')
            if suc:
                break
        self.serial_thread = threading.Thread(target=self.read_serial_data)
        self.serial_thread.start()
        

    def data_filter_setting_callback(self, msg):
        self.data_filter_setting = msg
        if self.size_maf != msg.maf_buffer_size:
            self.size_maf = msg.maf_buffer_size
            self.force3d_buffer = np.zeros((self.size_maf,3))
            self.torque3d_buffer = np.zeros((self.size_maf,3))
            self.loadcell_weight_buffer = np.zeros((self.size_maf,2))

        # self.force3d_buffer
    

    # def LPF_state_callback(self, msg):
    #     self.LPF_state = msg
    #     # self.get_logger().info(f'LPF_state: {self.LPF_state.data}')

    # def MAF_state_callback(self, msg):
    #     self.MAF_state = msg
    #     # self.get_logger().info(f'MAF_state: {self.MAF_state.data}')

    def publishall(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fts'
        msg.wrench.force.x = self.force3d.squeeze()[0]
        msg.wrench.force.y = self.force3d.squeeze()[1]
        msg.wrench.force.z = self.force3d.squeeze()[2]
        msg.wrench.torque.x = self.torque3d.squeeze()[0]
        msg.wrench.torque.y = self.torque3d.squeeze()[1]
        msg.wrench.torque.z = self.torque3d.squeeze()[2]
        self.fts_publisher.publish(msg)

        msg = LoadcellState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'loadcell'
        msg.stress.append(self.loadcell_weight.squeeze()[0])
        msg.stress.append(self.loadcell_weight.squeeze()[1])
        # msg.output_voltage = self.loadcell_weight[1]

        self.loadcell_publisher.publish(msg)
        
    def read_serial_data(self):
        try:
            self.get_logger().info('Start serial reading')
            while True:
                # 시리얼 데이터 읽기
                try:
                    # serial_data = ser.readline()  # binary(ASCII)
                    serial_data = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                except UnicodeDecodeError as e:
                    self.get_logger().warning('Error decoding data')
                    serial_data = ''

                # 수신된 데이터가 비어있지 않으면 출력
                if serial_data:
                    parsing_data = self.parse_serial_data(serial_data)
                    if parsing_data == None:
                        continue
                    else:
                        try:
                            if self.data_filter_setting.set_lpf == True:
                                self.force3d = (1-self.data_filter_setting.lpf_weight) * self.force3d + self.data_filter_setting.lpf_weight * (np.asarray(parsing_data[0:3]) - self.offset_force3d)
                                self.torque3d = (1-self.data_filter_setting.lpf_weight) * self.torque3d + self.data_filter_setting.lpf_weight * (np.asarray(parsing_data[3:6]) - self.offset_torque3d)
                                self.loadcell_weight = (1-self.data_filter_setting.lpf_weight) * self.loadcell_weight + self.data_filter_setting.lpf_weight * (np.asarray(parsing_data[6:8]) - self.offset_loadcell_weight)
                                # self.get_logger().info(f'LPF: {self.force3d}')
                                
                            else:   # raw data
                                # self.get_logger().info(f'Raw(offset X): {np.asarray(parsing_data[0:3])}')
                                self.force3d = np.asarray(parsing_data[0:3]) - np.asarray(self.offset_force3d)
                                self.torque3d = np.asarray(parsing_data[3:6]) - np.asarray(self.offset_torque3d)
                                self.loadcell_weight = np.asarray(parsing_data[6:8]) - np.asarray(self.offset_loadcell_weight)
                                # self.get_logger().info(f'Raw(offset O): {self.force3d}')
                                
                            self.force3d = np.reshape(self.force3d, (1,3))
                            self.torque3d = np.reshape(self.torque3d, (1,3))
                            self.loadcell_weight = np.reshape(self.loadcell_weight, (1,2))
                            # self.force3d = np.expand_dims(np.reshape(self.force3d, (1,3)), axis=0)
                            # self.torque3d = np.expand_dims(np.reshape(self.torque3d, (1,3)), axis=0)
                            # self.loadcell_weight = np.expand_dims(np.reshape(self.loadcell_weight, (1,2)), axis=0)
                            

                            ############################################################################################
                            if self.data_filter_setting.set_maf == True:
                                # self.get_logger().info(f'MAF: {self.force3d} / shape={self.force3d.shape}')
                                self.force3d = self.MovingAverageFilter(self.force3d_buffer, self.force3d)
                                self.torque3d = self.MovingAverageFilter(self.torque3d_buffer, self.torque3d)
                                self.loadcell_weight = self.MovingAverageFilter(self.loadcell_weight_buffer, self.loadcell_weight)
                                # self.get_logger().info(f'MAF result_dim {self.force3d.ndim} / shape={self.force3d.shape}')
                                
                                # Moving average buffer set
                                self.force3d_buffer = np.delete(self.force3d_buffer, 0, axis=0)
                                self.force3d_buffer = np.append(self.force3d_buffer, self.force3d, axis=0)
                                self.torque3d_buffer = np.delete(self.torque3d_buffer, 0, axis=0)
                                self.torque3d_buffer = np.append(self.torque3d_buffer, self.torque3d, axis=0)
                                self.loadcell_weight_buffer = np.delete(self.loadcell_weight_buffer, 0, axis=0)
                                self.loadcell_weight_buffer = np.append(self.loadcell_weight_buffer, self.loadcell_weight, axis=0)

                            else:
                                self.force3d_buffer = np.delete(self.force3d_buffer, 0, axis=0)
                                self.force3d_buffer = np.append(self.force3d_buffer, self.force3d, axis=0)
                                self.torque3d_buffer = np.delete(self.torque3d_buffer, 0, axis=0)
                                self.torque3d_buffer = np.append(self.torque3d_buffer, self.torque3d, axis=0)
                                self.loadcell_weight_buffer = np.delete(self.loadcell_weight_buffer, 0, axis=0)
                                self.loadcell_weight_buffer = np.append(self.loadcell_weight_buffer, self.loadcell_weight, axis=0)

                            # self.get_logger().info(f'Final: {self.force3d}')
                            # self.get_logger().info(f'==========================================')

                            self.publishall()
                            
                        except ValueError as e:
                            self.get_logger().warning(f'(read) Error : {e}')
                            pass
                        
        except KeyboardInterrupt:
            self.get_logger().warning('Keyboard Interrupt')
        finally:
            # 시리얼 포트 닫기
            # self.get_logger().info('Close serial reading')
            self.ser.close()

    def set_zero(self, avg_num=100):
        try:
            force_3d = []
            torque_3d = []
            lc = []
            count = 0

            # arduino data will be error in few seconds at start point
            # So must flushing few data (N)
            N = 30
            for i in range(N):
                serial_data = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                if serial_data:
                    serial_data = ''
                if i == N-1:
                    self.get_logger().warning(f'deleting {N} data is flushing at the start point')

            while count < avg_num:
                # 시리얼 데이터 읽기
                try:
                    serial_data = self.ser.readline().decode('utf-8', errors='ignore').rstrip()
                    # 수신된 데이터가 비어있지 않으면 출력
                    if serial_data:
                        parsing_data = self.parse_serial_data(serial_data)
                        if parsing_data == None:
                            continue
                        else:
                            # self.get_logger().info(f'parsing data = {parsing_data}')
                            try:
                                if np.any(np.array(parsing_data)==0):
                                    continue
                                else:
                                    # self.get_logger().info(f'[{count}]parsing data = {parsing_data}')
                                    force_3d.append(parsing_data[0:3])
                                    torque_3d.append(parsing_data[3:6])
                                    lc.append(parsing_data[6:8])
                                    count = count + 1
                                # self.publishall()
                            except ValueError as e:
                                self.get_logger().warning(f'(set_zero) Error {e}')
                                return 0
                            finally:
                                pass
                except UnicodeDecodeError as e:
                    self.get_logger().warning(f'Error decoding data: {e}')
                    serial_data = ''
                finally:
                    pass

            # self.offset_force3d = int(np.array(force_3d).mean(axis=0))
            # self.offset_torque3d = int(np.array(torque_3d).mean(axis=0))
            # self.offset_loadcell_weight = int(np.array(lc).mean(axis=0))

            self.offset_force3d = np.asarray(force_3d).mean(axis=0).astype(int)
            self.offset_torque3d = np.asarray(torque_3d).mean(axis=0).astype(int)
            self.offset_loadcell_weight = np.asarray(lc).mean(axis=0).astype(int)
            
            self.get_logger().warning(f'offset_force3d : {self.offset_force3d}')
            self.get_logger().warning(f'offset_torque3d: {self.offset_torque3d}')
            self.get_logger().warning(f'offset_loadcell_weight: {self.offset_loadcell_weight}')

            len_f3d = len(self.offset_force3d)
            len_t3d = len(self.offset_torque3d)
            len_lc = len(self.offset_loadcell_weight)
            if len_f3d !=3 or len_t3d !=3 or len_lc !=2:
                return 0
        except KeyboardInterrupt:
            self.get_logger().warning('Keyboard Interrupt')
            return 0
        finally:
            return 1
    
    def parse_serial_data(self, str: str):
        stx = '/'
        etx = ';'
        sidx = str.find(stx)
        eidx = str.find(etx)

        if sidx == -1 or eidx == -1 or sidx >= eidx:
            return None
        try:
            data_part = str[sidx+1:eidx]
            data_list = data_part.split(',')
            data_list_float = [float(item) for item in data_list]
            return data_list_float
        except Exception as e:
            self.get_logger().warning(f'Error while parsing input string: {e}')
            return None


    def MovingAverageFilter(self, prev_data, new_data):
        # self.get_logger().info(f'[{self.buffer_count}] prev: {prev_data} / new: {new_data}')
        # if len(prev_data[0]) != len(new_data):
        #     return
        prev_data = np.delete(prev_data, 0, axis=0)
        prev_data = np.append(prev_data, new_data, axis=0)

        data = prev_data
        avg = sum(data) / len(data)
        avg = np.expand_dims(avg, axis=0)
        self.buffer_count = self.buffer_count +1
        return avg

    # def set_zero_callback(self, request, response):
    #     force_x = []
    #     force_y = []
    #     force_z = []
    #     torque_x = []
    #     torque_y = []
    #     torque_z = []
    #     lc_1 = []
    #     lc_2 = []


    #     self.force3d[0]
    #     self.force3d[1]
    #     self.force3d[2]
    #     self.torque3d[0]
    #     self.torque3d[1]
    #     self.torque3d[2]
        

def main(args=None):
    rclpy.init(args=args)
    try:
        serial_node = SerialNode()
        try:
            rclpy.spin(serial_node)
        except KeyboardInterrupt:
            serial_node.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            serial_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()