import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mujoco_pkg = get_package_share_directory("mujoco_pendulum")
    orion_pkg = get_package_share_directory("orion5_humble_description")

    urdf_path = os.path.join(orion_pkg, "urdf", "orion5.urdf")
    controllers_path = os.path.join(mujoco_pkg, "config", "orion5_controllers.yaml")

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_path,
        ],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller"],
        output="screen",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [
            ros2_control_node,
            joint_state_broadcaster,
            effort_controller,
            robot_state_pub,
            rviz_node,
        ]
    )
