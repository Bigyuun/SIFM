import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtCore import QTimer, QTextStream, Qt
from PyQt5 import QtCore
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from collections import deque
import numpy as np

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from custom_interfaces.msg import MotorCommand
from custom_interfaces.msg import MotorState

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.motor_command_publisher_ = self.create_publisher(MotorCommand, 'motor_command', QOS_RKL10V)
        
        self.fts_data = WrenchStamped()
        self.fts_subscriber = self.create_subscription(
            WrenchStamped,
            'fts_data_raw',
            self.read_fts_data,
            QOS_RKL10V
        )
        self.get_logger().info('fts_data subscriber is created.')

        self.motor_state = MotorState()
        self.motor_state_subscriber = self.create_subscription(
            MotorState,
            'motor_state',
            self.read_motor_state,
            QOS_RKL10V
        )
        self.get_logger().info('motor_state subscriber is created.')

    def read_fts_data(self, msg):
        self.fts_data = msg

    def read_motor_state(self, msg):
        self.motor_state = msg
        # pass

    def publish(self, msg):
        self.motor_command_publisher_.publish(msg)

class MyGUI(QWidget):
    def __init__(self, node):
        super().__init__()

        self.node = node
        self.layout_global = QVBoxLayout()

        '''
        @ autor DY
        @ note publisher list for command
        '''
        self.motor_pub_label_list = []
        self.motor_pub_line_edit_list = []
        self.motor_pub_button_list = []
        self.motor_pub_layout_list = []
        for i in range(2):  # make label, line_editor, push button for motor control
            self.motor_pub_layout_list.append(QHBoxLayout())
            self.motor_pub_label_list.append(QLabel(f'Motor #{i}'))
            self.motor_pub_line_edit_list.append(QLineEdit('0'))
            self.motor_pub_button_list.append(QPushButton('Publish'))

            # @ note  if you input 'i' into argument of function 'publish_message' directly,
            #         then all button will work about num=2, in this case
            if i == 0:
                self.motor_pub_button_list[i].clicked.connect(lambda : self.publish_message(num=0))
            elif i == 1:
                self.motor_pub_button_list[i].clicked.connect(lambda : self.publish_message(num=1))
        
        # @autor DY
        # lambda F for apply the funtion to all the list arugments)
        list(map(lambda x: x.setAlignment(Qt.AlignVCenter | Qt.AlignRight), self.motor_pub_label_list))
        list(map(lambda x: x.setFixedWidth(150), self.motor_pub_button_list))
        list(map(lambda x: x.setFixedHeight(30), self.motor_pub_button_list))
        list(map(lambda x: x.setFixedWidth(100), self.motor_pub_line_edit_list))
        list(map(lambda x: x.setFixedHeight(30), self.motor_pub_line_edit_list))
        
        for i in range(len(self.motor_pub_label_list)):
            try:
                self.motor_pub_layout_list[i].addWidget(self.motor_pub_label_list[i])
                self.motor_pub_layout_list[i].addWidget(self.motor_pub_line_edit_list[i])
                self.motor_pub_layout_list[i].addWidget(self.motor_pub_button_list[i])
                self.layout_global.addLayout(self.motor_pub_layout_list[i])
            finally:
                pass

        '''
        @ autor DY
        @ note subscriber list for monitoring
        '''
        self.fts_sub_label_list = []
        self.fts_sub_line_edit_list = []
        self.fts_sub_layout_list = []
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_fx'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_fy'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_fz'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_tx'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_ty'))
        self.fts_sub_label_list.append(QLabel('force_torque_sensor_tz'))

        for i in range (len(self.fts_sub_label_list)):  # 6 (force 3d, torque 3d)
            try:
                self.fts_sub_layout_list.append(QHBoxLayout())
                self.fts_sub_line_edit_list.append(QLineEdit('0'))

                self.fts_sub_layout_list[i].addWidget(self.fts_sub_label_list[i])
                self.fts_sub_layout_list[i].addWidget(self.fts_sub_line_edit_list[i])
                self.layout_global.addLayout(self.fts_sub_layout_list[i])
            finally:
                pass

        list(map(lambda x: x.setReadOnly(True), self.fts_sub_line_edit_list))


        '''
        @ author DY
        @ note Timer objects
        '''
        # ros node spin
        self.timer_ros_node = QTimer(self)
        self.timer_ros_node.timeout.connect(self.node_spin_once)
        self.timer_ros_node.start(10)  # 10 밀리초 주기로 타이머 실행
        # fts_data update
        self.timer_plot = QTimer(self)
        self.timer_plot.timeout.connect(self.update_fts)
        self.timer_plot.start(33)

        # Matplotlib graph
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.layout_global.addWidget(self.canvas)
        self.data_y = np.zeros((6, 50))  # 초기 데이터 설정 (6개의 데이터, 각각 100개의 요소)
        # 그래프 초기화
        self.lines = [self.ax.plot([], [], label=f'Data {i}')[0] for i in range(6)]
        self.ax.legend()
        # 애니메이션 시작
        self.animation = FuncAnimation(self.figure, self.update_fts_plot, interval=25)

        self.setLayout(self.layout_global)



    def create_subscriber_gui(self,
                              label_text='label',
                              *,
                              line_edit_text='0'
                              ):
        label = QLabel(f"{label_text}")
        line_edit = QLineEdit(f"{line_edit_text}")
        pass

    def publish_message(self, num):
        message = MotorCommand()
        cmd_val=[]
        try:
            if len(self.node.motor_state.actual_position) ==0:
                self.node.get_logger().warning(f'command val : {cmd_val}')
                self.node.get_logger().warning(f'motor_state does not exit. Check the connection')
                return
            else:
                cmd_val.extend(self.node.motor_state.actual_position)
                cmd_val[num] = cmd_val[num] + (int(self.motor_pub_line_edit_list[num].text()))
                message.target_position.extend(cmd_val)
        except Exception as e:
            self.node.get_logger().warning(f'F:publish_message() -> {e}')
            return
        self.node.publish(message)
        self.node.get_logger().info(f'motor #{num} -> {cmd_val} command update completely.')

    def node_spin_once(self):
        rclpy.spin_once(self.node)

    def update_fts(self):
        try:

            
            self.fts_sub_line_edit_list[0].setText(str(self.node.fts_data.wrench.force.x))
            self.fts_sub_line_edit_list[1].setText(str(self.node.fts_data.wrench.force.y))
            self.fts_sub_line_edit_list[2].setText(str(self.node.fts_data.wrench.force.z))
            self.fts_sub_line_edit_list[3].setText(str(self.node.fts_data.wrench.torque.x))
            self.fts_sub_line_edit_list[4].setText(str(self.node.fts_data.wrench.torque.y))
            self.fts_sub_line_edit_list[5].setText(str(self.node.fts_data.wrench.torque.z))
            
            # for i in range(len(self.fts_sub_line_edit_list)):   # for matplotlib graph
            #     self.data_y.append(self.node.fts_data.wrench.torque.x)
            self.data_x = [i for i in range(len(self.data_y))]

        except Exception as e:
            self.node.get_logger().warning(f'F:update_fts() -> {e}')

        return 1
    
    def update_fts_plot(self, frame):
        try:
            new_data = np.zeros(6)
            new_data[0] = (self.node.fts_data.wrench.force.x)
            new_data[1] = (self.node.fts_data.wrench.force.y)
            new_data[2] = (self.node.fts_data.wrench.force.z)
            new_data[3] = (self.node.fts_data.wrench.torque.x)
            new_data[4] = (self.node.fts_data.wrench.torque.y)
            new_data[5] = (self.node.fts_data.wrench.torque.z)

            self.data_y = np.roll(self.data_y, shift=-1, axis=1)
            self.data_y[:, -1] = new_data

            for i, line in enumerate(self.lines):
                line.set_data(range(len(self.data_y[i])), self.data_y[i])
            

            # 최댓값과 최솟값을 찾아 축의 범위 설정
            min_val = np.min(self.data_y)
            max_val = np.max(self.data_y)
            margin = 1000  # 여분의 여백 설정
            self.ax.set_xlim(0, len(self.data_y[0]) - 1)
            self.ax.set_ylim(min_val - margin, max_val + margin)

            # self.ax.clear()
            # self.ax.plot(self.data_x, self.data_y)
            # self.ax.plot(self.data_x, self.data_y)
            # self.ax.plot(self.data_x, self.data_y)
            # self.ax.plot(self.data_x, self.data_y)
            # self.ax.plot(self.data_x, self.data_y)
            # self.ax.plot(self.data_x, self.data_y)
            # self.ax.set_title('Real-time Force-Torque data')
            # self.ax.set_xlabel('time')
            # self.ax.set_ylabel('N-m')
            return self.lines
        except Exception as e:
            self.node.get_logger().warning(f'F:update_fts_plot() -> {e}')


def main():
    rclpy.init(args=None)
    node = GUINode()
    app = QApplication(sys.argv)
    gui = MyGUI(node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()

