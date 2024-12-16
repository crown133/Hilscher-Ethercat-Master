#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from libertech_interface_msgs.msg import HybridJointState
from std_msgs.msg import Header
import tkinter as tk

class HybridJointCmdPublisher(Node):

    def __init__(self):
        super().__init__('hybrid_joint_cmd_publisher')

        # 预定义各个部分的关节名称
        self.leg_joint_names = ["waist_yaw",'leg_left_HTAA', 'leg_left_HAA', 'leg_left_HFE', 'leg_left_KFE', 'leg_left_AAA', 'leg_left_AFE',
            'leg_right_HTAA', 'leg_right_HAA', 'leg_right_HFE', 'leg_right_KFE', 'leg_right_AAA', 'leg_right_AFE',
        ]
        self.arm_left_joint_names = ['arm_left_shoulder_pitch_joint', 'arm_left_shoulder_roll_joint', 'arm_left_shoulder_yaw_joint',
            'arm_left_elbow_pitch_joint', 'arm_left_elbow_roll_joint', 'arm_left_wrist_pitch_joint', 'arm_left_wrist_yaw_joint',
        ]
        self.arm_right_joint_names = ['arm_right_shoulder_pitch_joint', 'arm_right_shoulder_roll_joint', 'arm_right_shoulder_yaw_joint',
            'arm_right_elbow_pitch_joint', 'arm_right_elbow_roll_joint', 'arm_right_wrist_pitch_joint', 'arm_right_wrist_yaw_joint',
        ]

        self.body_joint_names = self.leg_joint_names + self.arm_left_joint_names + self.arm_right_joint_names
        self.head_joint_names = ['head_yaw_joint', 'head_pitch_joint']
        self.hand_left_joint_names = [
            'hand_left_f11_joint', 'hand_left_f12_joint', 'hand_left_f13_joint',
            'hand_left_f21_joint', 'hand_left_f22_joint', 'hand_left_f23_joint',
            'hand_left_f31_joint', 'hand_left_f32_joint',
            'hand_left_f41_joint', 'hand_left_f42_joint',
            'hand_left_f51_joint', 'hand_left_f52_joint', 'hand_left_f53_joint',
        ]
        self.hand_right_joint_names = [
            'hand_right_f11_joint', 'hand_right_f12_joint', 'hand_right_f13_joint',
            'hand_right_f21_joint', 'hand_right_f22_joint', 'hand_right_f23_joint',
            'hand_right_f31_joint', 'hand_right_f32_joint',
            'hand_right_f41_joint', 'hand_right_f42_joint',
            'hand_right_f51_joint', 'hand_right_f52_joint', 'hand_right_f53_joint',
        ]

        # 初始化Publisher
        self.body_publisher = self.create_publisher(HybridJointState, '/hybrid_joint_cmd_elmo', 10)
        self.head_publisher = self.create_publisher(HybridJointState, '/hybrid_joint_cmd_head', 10)
        self.hand_left_publisher = self.create_publisher(HybridJointState, '/hybrid_joint_cmd_hand_left', 10)
        self.hand_right_publisher = self.create_publisher(HybridJointState, '/hybrid_joint_cmd_hand_right', 10)

        # 初始化GUI
        self.init_gui()

    def init_gui(self):
        self.root = tk.Tk()
        self.root.title("Hybrid Joint Command Controller")

        # 使用Grid布局滑动条
        self.sliders = {}
        row_counter = 0

        for joint_names in [self.leg_joint_names, self.arm_left_joint_names, self.arm_right_joint_names, self.head_joint_names, self.hand_left_joint_names, self.hand_right_joint_names]:
            frame = tk.Frame(self.root)
            frame.pack(side=tk.LEFT, fill=tk.Y)
            for joint_name in joint_names:
                label = tk.Label(frame, text=joint_name)
                label.pack(side=tk.TOP)
                slider = tk.Scale(frame, from_=-0.8, to=0.8, resolution=0.01, orient=tk.HORIZONTAL)
                slider.pack(side=tk.TOP, fill=tk.X)
                self.sliders[joint_name] = slider
                row_counter += 1

        # 创建定时器来发布消息
        self.root.after(10, self.publish_joint_commands)
        self.root.mainloop()

    # def init_gui(self):
    #     self.root = tk.Tk()
    #     self.root.title("Hybrid Joint Command Controller")

    #     # 将滑动条分组并放置在不同的列中
    #     self.sliders = {}
    #     groups = [
    #         ("Body", self.body_joint_names),
    #         ("Head", self.head_joint_names),
    #         ("Left Hand", self.hand_left_joint_names),
    #         ("Right Hand", self.hand_right_joint_names)
    #     ]

    #     for i, (group_name, joint_names) in enumerate(groups):
    #         frame = tk.Frame(self.root)
    #         frame.grid(row=0, column=i, padx=10, pady=10)
    #         label = tk.Label(frame, text=group_name, font=("Arial", 12, "bold"))
    #         label.pack()
    #         for joint_name in joint_names:
    #             subframe = tk.Frame(frame)
    #             subframe.pack(fill=tk.X)
    #             sublabel = tk.Label(subframe, text=joint_name)
    #             sublabel.pack(side=tk.LEFT)
    #             slider = tk.Scale(subframe, from_=-1.0, to=1.0, resolution=0.01, orient=tk.HORIZONTAL)
    #             slider.pack(side=tk.RIGHT, fill=tk.X)
    #             self.sliders[joint_name] = slider

    #     # 创建定时器来发布消息
    #     self.root.after(1000, self.publish_joint_commands)
    #     self.root.mainloop()

    def create_joint_state_msg(self, joint_names):
        msg = HybridJointState()
        msg.name = joint_names
        msg.pos = [self.sliders[joint_name].get() for joint_name in joint_names]
        msg.vel = [self.sliders[joint_name].get() for joint_name in joint_names]
        msg.tau = [self.sliders[joint_name].get() for joint_name in joint_names]
        msg.kp = [self.sliders[joint_name].get() * 10.0 for joint_name in joint_names]
        msg.kd = [self.sliders[joint_name].get() * 5.0 for joint_name in joint_names]
        return msg

    def publish_joint_commands(self):
        # 生成每个部件的JointState消息
        body_msg = self.create_joint_state_msg(self.body_joint_names)
        head_msg = self.create_joint_state_msg(self.head_joint_names)
        hand_left_msg = self.create_joint_state_msg(self.hand_left_joint_names)
        hand_right_msg = self.create_joint_state_msg(self.hand_right_joint_names)

        # 设置header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        body_msg.header = header
        head_msg.header = header
        hand_left_msg.header = header
        hand_right_msg.header = header

        # 发布消息
        self.body_publisher.publish(body_msg)
        self.head_publisher.publish(head_msg)
        self.hand_left_publisher.publish(hand_left_msg)
        self.hand_right_publisher.publish(hand_right_msg)

        # self.get_logger().info("Published joint commands.")

        # 继续定时发布
        self.root.after(10, self.publish_joint_commands)
        
    def on_close(self):
        self.get_logger().info("Shutting down GUI and ROS node...")
        self.destroy_node()  # 销毁ROS节点
        self.root.destroy()  # 关闭GUI

def main(args=None):
    rclpy.init(args=args)
    try:
        node = HybridJointCmdPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
        node.on_close()  # 调用关闭方法
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
