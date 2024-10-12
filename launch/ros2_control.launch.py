from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("md80_hardware_interface"),
                    "urdf/ros2_control.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("md80_hardware_interface"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        emulate_tty=True,
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "controller_manager",
        ],
    )

    return LaunchDescription(
        [
            robot_state_pub_node,
            control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller,
        ]
    )
