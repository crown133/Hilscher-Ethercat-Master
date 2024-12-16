#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLineEdit, QPushButton, QLabel, QCheckBox
from PyQt5.QtCore import Qt

import numpy as np
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.publisher_ = self.create_publisher(Float64, '/motor_pos', 10)
        self.flag_ = True  # True for slider mode, False for manual input mode

    def publish(self, value):
        msg = Float64()
        msg.data = value
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Published: {value}")

class FloatSlider(QSlider):
    def __init__(self, decimals=3, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.decimals = decimals  # 设置小数位数
        self.int_scale = 10**decimals  # 用于映射为整数值的倍数
        self.setOrientation(Qt.Horizontal)
        self.setMinimum(0)
        self.setMaximum(self.int_scale)  # 最大值变为 `10^decimals`

    def value(self):
        return super().value() / self.int_scale  # 映射为浮点数值

    def setFloatValue(self, val):
        super().setValue(int(val * self.int_scale))  # 将浮点数值映射为整数值
        
class ControlWidget(QWidget):
    def __init__(self, motor_node):
        super().__init__()
        self.motor_node = motor_node

        # 创建布局
        layout = QVBoxLayout()

        # 标题
        self.title = QLabel('Motor Position Controller', self)
        layout.addWidget(self.title)

        # 滑动条
        # self.slider = FloatSlider(decimals=5)
        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setMinimum(-10*10**8)
        self.slider.setMaximum(10*10**8)
        self.slider.setValue(0)
        layout.addWidget(self.slider)

        # 显示滑动条数值的标签
        self.slider_value_label = QLabel(f"Slider Value: {self.slider.value()}", self)
        layout.addWidget(self.slider_value_label)

        # 输入框
        self.input_field = QLineEdit(self)
        layout.addWidget(self.input_field)

        # 发送按钮
        self.send_button = QPushButton('Send Input Value', self)
        layout.addWidget(self.send_button)

        # 模式选择复选框
        self.mode_checkbox = QCheckBox('Use Slider Mode', self)
        self.mode_checkbox.setChecked(True)
        layout.addWidget(self.mode_checkbox)

        # 绑定信号和槽函数
        self.slider.valueChanged.connect(self.update_slider_value)
        self.send_button.clicked.connect(self.send_input_value)
        self.mode_checkbox.stateChanged.connect(self.update_mode)

        self.setLayout(layout)
    
    def update_slider_value(self):
        value = self.slider.value() / 10**8
        # print(value)
        self.slider_value_label.setText(f"Slider Value: {value:.1f}")

        # 如果 flag_ 为 True，则滑动条控制发布
        if self.motor_node.flag_:
            self.motor_node.publish(value)

    def send_input_value(self):
        try:
            value = float(self.input_field.text())
            # 如果 flag_ 为 False，则手动输入控制发布
            if not self.motor_node.flag_:
                self.motor_node.publish(value)
                # duration = 1.0
                # steps = int(duration / 0.015)
                
                # trajectory = self.generate_polynomial_trajectory(target=value, duration=duration, steps=steps)
                # for point in trajectory:
                #     self.motor_node.publish(point)
                #     time.sleep(duration/steps)
                    
        except ValueError:
            self.input_field.setText("Invalid Input")

    def update_mode(self, state):
        # 切换模式
        self.motor_node.flag_ = True if state == Qt.Checked else False
        
    def generate_polynomial_trajectory(self, target, duration=2.0, steps=100):
        """
        Generates a 5th-order polynomial trajectory from 0 to target.
        :param target: The target position.
        :param duration: The duration of the trajectory in seconds.
        :param steps: The number of steps for the trajectory.
        :return: A list of trajectory points.
        """
        # Initial and final conditions for position, velocity, and acceleration
        t0, tf = 0, duration
        p0, pf = 0, target
        v0, vf = 0, 0
        a0, af = 0, 0

        # Time vector
        t = np.linspace(t0, tf, steps)
        
        # Coefficients for the 5th-order polynomial
        a = np.zeros(6)
        a[0] = p0
        a[1] = v0
        a[2] = 0.5 * a0
        a[3] = (20*pf - 20*p0 - (8*vf + 12*v0)*tf - (3*a0 - af)*tf**2) / (2*tf**3)
        a[4] = (30*p0 - 30*pf + (14*vf + 16*v0)*tf + (3*a0 - 2*af)*tf**2) / (2*tf**4)
        a[5] = (12*pf - 12*p0 - (6*vf + 6*v0)*tf - (a0 - af)*tf**2) / (2*tf**5)

        # Generate trajectory points
        trajectory = a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5
        return trajectory            
            
def main(args=None):
    rclpy.init(args=args)

    # 初始化 ROS 2 节点
    motor_node = MotorControlNode()

    # 创建 PyQt 应用程序
    app = QApplication(sys.argv)
    control_widget = ControlWidget(motor_node)
    control_widget.setWindowTitle("Motor Control with PyQt")
    control_widget.show()

    try:
        sys.exit(app.exec_())
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
