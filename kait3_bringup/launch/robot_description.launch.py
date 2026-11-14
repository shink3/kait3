from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description() -> LaunchDescription:
    mock_hardware_arg = DeclareLaunchArgument(
        "mock_hardware", default_value="false",
        description="Use mock hardware interface if true"
    )

    xacro_path = PathJoinSubstitution([
        FindPackageShare("kait3_description"),
        "urdf",
        "robot.urdf.xacro"
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
            " ",
            "mock_hardware:=",
            LaunchConfiguration("mock_hardware")
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    return LaunchDescription([
        mock_hardware_arg,
        robot_state_publisher
    ])




