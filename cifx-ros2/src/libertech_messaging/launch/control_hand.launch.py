from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    use_hand_left = LaunchConfiguration("use_hand_left")
    use_hand_right = LaunchConfiguration("use_hand_right")

    action_file = PathJoinSubstitution(
        [FindPackageShare("libertech_hand_driver"), "config", "actions.yaml"]
    )

    hand_left_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("libertech_hand_driver"),
                "/launch/hand_ros_driver.launch.py",
            ]
        ),
        launch_arguments={
            "hand_type": "left",
            "hand_ip": "192.168.1.200",
            "action_file": action_file,
            "dummy": "false",
        }.items(),
        condition=IfCondition(use_hand_left),
    )

    hand_right_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("libertech_hand_driver"),
                "/launch/hand_ros_driver.launch.py",
            ]
        ),
        launch_arguments={
            "hand_type": "right",
            "hand_ip": "192.168.1.201",
            "action_file": action_file,
            "dummy": "false",
        }.items(),
        condition=IfCondition(use_hand_right),
    )

    nodes_to_start = [
        hand_left_driver_launch,
        hand_right_driver_launch,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hand_left",
            default_value="true",
            description="Use hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hand_right",
            default_value="true",
            description="Use hand.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
