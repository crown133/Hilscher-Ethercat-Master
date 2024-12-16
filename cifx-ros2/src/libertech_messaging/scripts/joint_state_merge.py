#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMergeNode(Node):
    def __init__(self):
        super().__init__('joint_state_merge_node')

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
            'hand_left_f11_joint', 'hand_left_f12_joint', 'hand_left_f13_joint', 'hand_left_f14_joint',
            'hand_left_f21_joint', 'hand_left_f22_joint', 'hand_left_f23_joint', 'hand_left_f24_joint',
            'hand_left_f31_joint', 'hand_left_f32_joint', 'hand_left_f33_joint', 'hand_left_f34_joint',
            'hand_left_f41_joint', 'hand_left_f42_joint', 'hand_left_f43_joint', 'hand_left_f44_joint',
            'hand_left_f51_joint', 'hand_left_f52_joint', 'hand_left_f53_joint', 'hand_left_f54_joint'
        ]
        self.hand_right_joint_names = [
            'hand_right_f11_joint', 'hand_right_f12_joint', 'hand_right_f13_joint', 'hand_right_f14_joint',
            'hand_right_f21_joint', 'hand_right_f22_joint', 'hand_right_f23_joint', 'hand_right_f24_joint',
            'hand_right_f31_joint', 'hand_right_f32_joint', 'hand_right_f33_joint', 'hand_right_f34_joint',
            'hand_right_f41_joint', 'hand_right_f42_joint', 'hand_right_f43_joint', 'hand_right_f44_joint',
            'hand_right_f51_joint', 'hand_right_f52_joint', 'hand_right_f53_joint', 'hand_right_f54_joint'
        ]
        
        # 初始化消息
        self.body_msg = self.create_joint_state_msg(self.body_joint_names)
        self.head_msg = self.create_joint_state_msg(self.head_joint_names)
        self.hand_left_msg = self.create_joint_state_msg(self.hand_left_joint_names)
        self.hand_right_msg = self.create_joint_state_msg(self.hand_right_joint_names)

        # 订阅消息
        self.create_subscription(JointState, '/joint_states_body', self.body_callback, 10)
        self.create_subscription(JointState, '/joint_states_head', self.head_callback, 10)
        self.create_subscription(JointState, '/joint_states_hand_left', self.hand_left_callback, 10)
        self.create_subscription(JointState, '/joint_states_hand_right', self.hand_right_callback, 10)
        
        # 发布合并后的 JointState 消息
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state_view_pub = self.create_publisher(JointState, '/joint_states_view', 10)

        # 设置定时器回调函数
        self.create_timer(0.001, self.timer_callback)  # 1000Hz
        
        self.count_view = 0

    def create_joint_state_msg(self, joint_names):
        msg = JointState()
        msg.name = joint_names
        msg.position = [0.0] * len(joint_names)
        msg.velocity = [0.0] * len(joint_names)
        msg.effort = [0.0] * len(joint_names)
        return msg
    
    def body_callback(self, msg):
        self.body_msg.position = msg.position
        self.body_msg.velocity = msg.velocity
        self.body_msg.effort = msg.effort

    def head_callback(self, msg):
        self.head_msg.position = msg.position
        self.head_msg.velocity = msg.velocity
        self.head_msg.effort = msg.effort

    def hand_left_callback(self, msg):
        self.hand_left_msg.position = msg.position
        self.hand_left_msg.velocity = msg.velocity
        self.hand_left_msg.effort = msg.effort

    def hand_right_callback(self, msg):
        self.hand_right_msg.position = msg.position
        self.hand_right_msg.velocity = msg.velocity
        self.hand_right_msg.effort = msg.effort

    def timer_callback(self):
        # 创建合并后的 JointState 消息
        combined_msg = JointState()
        combined_msg.header.stamp = self.get_clock().now().to_msg()

        # 合并关节名称
        combined_msg.name = self.body_msg.name + self.head_msg.name + self.hand_left_msg.name + self.hand_right_msg.name
        combined_msg.position = self.body_msg.position + self.head_msg.position + self.hand_left_msg.position + self.hand_right_msg.position
        combined_msg.velocity = self.body_msg.velocity + self.head_msg.velocity + self.hand_left_msg.velocity + self.hand_right_msg.velocity
        combined_msg.effort = self.body_msg.effort + self.head_msg.effort + self.hand_left_msg.effort + self.hand_right_msg.effort

        # 发布合并后的消息
        self.joint_state_pub.publish(combined_msg)

        if self.count_view % 10 == 0:
            self.joint_state_view_pub.publish(combined_msg)

        self.count_view += 1
        if self.count_view >= 1e9:  # 假设你认为10亿是一个合理的重置点
            self.count_view = 0


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
