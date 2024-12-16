#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from libertech_interface_msgs.msg import HybridJointState
from std_msgs.msg import Header
import random

class HybridJointCmdPublisher(Node):

    def __init__(self):
        super().__init__('hybrid_joint_cmd_publisher')

        # 预定义各个部分的关节名称
        self.body_joint_names = [
            "waist_yaw",'leg_left_HTAA', 'leg_left_HAA', 'leg_left_HFE', 'leg_left_KFE', 'leg_left_AAA', 'leg_left_AFE',
            'leg_right_HTAA', 'leg_right_HAA', 'leg_right_HFE', 'leg_right_KFE', 'leg_right_AAA', 'leg_right_AFE',
            'arm_left_shoulder_pitch_joint', 'arm_left_shoulder_roll_joint', 'arm_left_shoulder_yaw_joint',
            'arm_left_elbow_pitch_joint', 'arm_left_elbow_roll_joint', 'arm_left_wrist_pitch_joint', 'arm_left_wrist_yaw_joint',
            'arm_right_shoulder_pitch_joint', 'arm_right_shoulder_roll_joint', 'arm_right_shoulder_yaw_joint',
            'arm_right_elbow_pitch_joint', 'arm_right_elbow_roll_joint', 'arm_right_wrist_pitch_joint', 'arm_right_wrist_yaw_joint'
        ]
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

        # 设置定时器，每秒发送一次消息
        self.timer = self.create_timer(1.0, self.publish_joint_commands)

    def create_joint_state_msg(self, joint_names):
        msg = HybridJointState()
        msg.name = joint_names
        msg.pos = [random.uniform(0.0, 0.5) for _ in joint_names]
        msg.vel = [random.uniform(-0.5, 0.5) for _ in joint_names]
        msg.tau = [random.uniform(-0.1, 0.1) for _ in joint_names]
        msg.kp = [random.uniform(0.0, 10.0) for _ in joint_names]
        msg.kd = [random.uniform(0.0, 5.0) for _ in joint_names]
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

        self.get_logger().info("Published joint commands.")

def main(args=None):
    rclpy.init(args=args)
    node = HybridJointCmdPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
