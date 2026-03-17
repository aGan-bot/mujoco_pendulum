from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_pendulum")
    sim_launch = os.path.join(pkg_share, "launch", "sim.launch.py")

    test_node = Node(
        package="mujoco_pendulum",
        executable="effort_test_node",
        output="screen",
        parameters=[
            {"torque_command": 20.0},
            {"publish_rate_hz": 100.0},
            {"step_mode": False},
        ],
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),
        test_node,
    ])
