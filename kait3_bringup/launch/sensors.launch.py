from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    config_file_path = PathJoinSubstitution([
        FindPackageShare("kait3_bringup"),
        "config",
        "MID360s.json"
    ]);

    realsense2_launch = PathJoinSubstitution([
       FindPackageShare("realsense2_camera"),
       "launch",
       "rs_launch.py"
    ])

    livox_launch = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "launch_ROS2",
        "MID360s.launch.py"
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense2_launch),
            launch_arguments={}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch),
            launch_arguments={
                "config_file": config_file_path,
                "rviz": "false",
            }.items()
        )
    ])
    
