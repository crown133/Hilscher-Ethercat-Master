from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 启动 ElmoTestNode (C++节点)
            Node(
                package="libertech_messaging",
                executable="elmo_test_node",
                name="elmo_test_node",
                output="screen",
                parameters=[
                    {
                        "recv_topic": "/motor_pos",
                        'ethercat_config_file': 'src/libertech_humanoid/ethercat_master/ethercat_device_configurator/example_config/setup.yaml'
                        # "ethercat_config_file": "src/libertech_humanoid/libertech_ethercat_driver/config/arm_right.yaml",
                    }
                ],
            ),
            # 启动 MotorControlNode (Python节点)
            Node(
                package="libertech_messaging",
                executable="motor_control_node.py",
                name="motor_control_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
