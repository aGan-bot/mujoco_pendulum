from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": open("src/mujoco_pendulum/urdf/pendulum.urdf").read()},
            "src/mujoco_pendulum/config/controllers.yaml"
        ],
        output="screen"
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
        parameters=[{
            "robot_description": open("src/mujoco_pendulum/urdf/pendulum.urdf").read()
        }],
        output="screen"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "src/mujoco_pendulum/config/pendulum.rviz"]  # kendi rviz config dosyan
    )
    
    return LaunchDescription([
        ros2_control_node,
        joint_state_broadcaster,
        effort_controller,
        robot_state_pub,
        rviz_node
    ])
