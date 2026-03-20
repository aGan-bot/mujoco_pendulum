from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _build_ff_node(context):
    pkg_share = get_package_share_directory("mujoco_pendulum")
    profile = LaunchConfiguration("profile").perform(context).strip().lower()
    if profile not in ("medium", "stable"):
        raise RuntimeError("Invalid 'profile'. Use 'medium' or 'stable'.")
    ff_cfg = os.path.join(pkg_share, "config", "tuning", f"pinocchio_ff_hold_{profile}.yaml")

    ff_node = Node(
        package="mujoco_pendulum",
        executable="pinocchio_ff_hold_node",
        output="screen",
        parameters=[ff_cfg],
    )
    return [ff_node]


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_pendulum")
    base_launch = os.path.join(pkg_share, "launch", "orion5_mujoco.launch.py")
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch))

    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="medium",
            description="FF hold tuning profile: medium | stable",
        ),
        sim,
        OpaqueFunction(function=_build_ff_node),
    ])
