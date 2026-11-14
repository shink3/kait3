from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file_path = PathJoinSubstitution([
        FindPackageShare("kait3_teleop"),
        "params",
        "teleop_parameters.yaml"
    ])

    deadzone_arg = DeclareLaunchArgument(
        "deadzone", default_value="0.05",
        description="Deadzone threshold to filter small analog stick drifts and jitters"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=params_file_path,
        description="Path to the parameter YAML file for the teleop node"
    )

    cmd_vel_arg = DeclareLaunchArgument(
        "cmd_vel", default_value="cmd_vel",
        description="Output velocity topic name"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        remappings=[("joy/set_feedback", "_joy_feedback")],
        parameters=[{
            "deadzone": LaunchConfiguration("deadzone")
        }]
    )

    ds4_teleop = Node(
        package="kait3_teleop",
        executable="ds4_teleop_node",
        name="ds4_teleop_node",
        output="screen",
        remappings=[("cmd_vel", LaunchConfiguration("cmd_vel"))],
        parameters=[
            LaunchConfiguration("params_file") 
        ]
    )

    return LaunchDescription([
        deadzone_arg,
        params_file_arg,
        cmd_vel_arg,
        joy_node,
        ds4_teleop
    ])




