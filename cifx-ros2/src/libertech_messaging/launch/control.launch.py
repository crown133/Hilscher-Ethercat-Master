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
    use_leg = LaunchConfiguration("use_leg")
    use_arm = LaunchConfiguration("use_arm")
    use_hand_left = LaunchConfiguration("use_hand_left")
    use_hand_right = LaunchConfiguration("use_hand_right")
    use_gripper_left = LaunchConfiguration("use_gripper_left")
    use_gripper_right = LaunchConfiguration("use_gripper_right")
    use_head = LaunchConfiguration("use_head")
    use_ft = LaunchConfiguration("use_ft")
    use_lidar = LaunchConfiguration("use_lidar")
    use_camera = LaunchConfiguration("use_camera")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    sim_ignition = LaunchConfiguration("sim_ignition")
    sim_ignition_world = LaunchConfiguration("sim_ignition_world")

    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_controllers = PathJoinSubstitution(
        # https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2/
        [
            FindPackageShare(description_package),
            "config",
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("libertech_teleoperation"), "rviz", "teleop.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_leg:=",
            use_leg,
            " ",
            "use_arm:=",
            use_arm,
            " ",
            "use_hand_left:=",
            use_hand_left,
            " ",
            "use_hand_right:=",
            use_hand_right,
            " ",
            "use_gripper_left:=",
            use_gripper_left,
            " ",
            "use_gripper_right:=",
            use_gripper_right,
            " ",
            "use_head:=",
            use_head,
            " ",
            "use_ft:=",
            use_ft,
            " ",
            "use_lidar:=",
            use_lidar,
            " ",
            "use_camera:=",
            use_camera,
            " ",
            "camera_width:=",
            camera_width,
            " ",
            "camera_height:=",
            camera_height,
            " ",
            "camera_fps:=",
            camera_fps,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "simulation_controllers:=",
            robot_controllers,
        ]
    )
    robot_description = {
        "robot_description": robot_description_content.perform(context)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            # {"joint_state_topic": "joint_states_view"} # 这个频率低一些，可以做到实时显示
        ],
        remappings=[
            (
                "/joint_states",
                "/joint_states_view",
            )  # 将 /joint_states 映射为 /joint_states_view
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    elmo_node = Node(
        package="libertech_messaging",
        executable="elmo_node",
        name="elmo_node",
        parameters=[{"dummy": False},
                    {"ethercat_config_file": "src/liberman_hardware/libertech_messaging/config/leg.yaml"}],
        output="screen",
        # emulate_tty=True,
    )
    # elmo_node = ExecuteProcess(
    #     cmd=[[
    #         '/home/ccs/ros2/humanoid_ws/src/libertech_humanoid/libertech_messaging/scripts/start_elmo_node.sh'
    #     ]],
    #     shell=True
    # )

    head_node = Node(
        package="libertech_messaging",
        executable="head_node",
        name="head_node",
        parameters=[{"dummy": False}],
        output="screen",
        # emulate_tty=True,
    )

    merge_node = Node(
        package="libertech_messaging",
        executable="joint_state_merge.py",
        name="merge_node",
        parameters=[],
        output="screen",
        # emulate_tty=True,
    )

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
        robot_state_publisher_node,
        rviz_node,
        elmo_node,
        # head_node,
        # hand_left_driver_launch,
        # hand_right_driver_launch,
        merge_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_leg",
            default_value="false",
            description="Use leg.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_arm",
            default_value="false",
            description="Use arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hand_left",
            default_value="false",
            description="Use hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_hand_right",
            default_value="false",
            description="Use hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper_left",
            default_value="false",
            description="Use gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper_right",
            default_value="false",
            description="Use gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_head",
            default_value="false",
            description="Use head.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_ft",
            default_value="false",
            description="Use ft.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_lidar",
            default_value="false",
            description="Use lidar.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description="Use head camera.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_width",
            default_value="640",
            description="camera width",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_height",
            default_value="480",
            description="camera height",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_fps",
            default_value="30",
            description="camera fps",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Use Gazebo Ignition for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition_world",
            default_value="world_empty.sdf",
            description="Gazebo Ignition world",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="libertech_humanoid_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="humanoid.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
