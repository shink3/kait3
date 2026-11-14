from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    xacro_path = PathJoinSubstitution([
        FindPackageShare("kait3_description"),
        "urdf",
        "robot.urdf.xacro"
    ])

    rviz_path = PathJoinSubstitution([
        FindPackageShare("kait3_description"),
        "rviz",
        "model.rviz"
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_path
        ]),
        value_type=str
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
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

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz2
    ])
