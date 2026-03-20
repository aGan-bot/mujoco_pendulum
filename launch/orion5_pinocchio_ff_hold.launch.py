from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_pendulum")
    base_launch = os.path.join(pkg_share, "launch", "orion5_mujoco.launch.py")
    ff_cfg = os.path.join(pkg_share, "config", "tuning", "pinocchio_ff_hold_medium.yaml")

    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch))

    ff_node = Node(
        package="mujoco_pendulum",
        executable="pinocchio_ff_hold_node",
        output="screen",
        parameters=[ff_cfg],
    )

    return LaunchDescription([
        sim,
        ff_node,
    ])

