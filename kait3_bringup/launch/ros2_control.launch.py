from launch import LaunchDescription
from launch.descriptions import executable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    controllers_file = PathJoinSubstitution([
        FindPackageShare("kait3_bringup"),
        "params",
        "ros2_controllers.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        output="screen"
    )

    emergency_stop_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["emergency_stop_controller"],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen"
    )

    return LaunchDescription([
        control_node,
        emergency_stop_controller_spawner,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ]) 
