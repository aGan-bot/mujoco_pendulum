from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mujoco_pendulum")
    base_launch = os.path.join(pkg_share, "launch", "orion5_mujoco.launch.py")
    medium_tuning = os.path.join(pkg_share, "config", "tuning", "home_hold_medium.yaml")

    gravity_comp_node = Node(
        package="mujoco_pendulum",
        executable="gravity_comp_relay_node",
        output="screen",
        parameters=[
            medium_tuning,
            {"gain": LaunchConfiguration("gain")},
            {"gain_vector": [1.0, 1.0, 1.05, 1.0, 1.0, 1.0]},
            {"q_ref": [0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0]},
            {"kp": [0.0, 0.0, 18.0, 0.0, 0.0, 0.0]},
            {"kd": [0.0, 0.0, 2.5, 0.0, 0.0, 0.0]},
            {"max_torque": LaunchConfiguration("max_torque")},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gain", default_value="1.0"),
        DeclareLaunchArgument("max_torque", default_value="200.0"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
        gravity_comp_node,
    ])
