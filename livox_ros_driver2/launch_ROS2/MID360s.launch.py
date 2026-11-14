import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    xfer_format_arg = DeclareLaunchArgument(
        "xfer_format", default_value="0",
        description="0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format"
    )

    multi_topic_arg = DeclareLaunchArgument(
        "multi_topic", default_value="1",
        description="0-All LiDARs share the same topic, 1-One LiDAR one topic"
    )

    publish_freq_arg = DeclareLaunchArgument(
        "publish_freq", default_value="10.0",
        description="Freqency of publish, 5.0, 10.0, 20.0, 50.0, etc."
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file", default_value="config/multi_MID360s_config.json",
        description="Path to the livox config JSON file relative to package share"
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Whether to execute rviz2"
    )

    rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file", default_value="config/display_point_cloud.rviz",
        description="Path to the rviz config file relative to package share"
    )

    xfer_format = {"xfer_format": LaunchConfiguration("xfer_format")}
    multi_topic = {"multi_topic": LaunchConfiguration("multi_topic")}
    data_src = {"data_src": 0}
    publish_freq = {"publish_freq": LaunchConfiguration("publish_freq")}
    output_data_type = {"output_data_type": 0}
    frame_id = {"frame_id": "livox_frame"}
    lvx_file_path = {"lvx_file_path": "/home/livox/livox_test.lcx"}
    user_config_path = {
        "user_config_path": 
        PathJoinSubstitution([FindPackageShare("livox_ros_driver2"), LaunchConfiguration("config_file")])
    }
    cmdline_bd_code = {"cmdline_input_bd_code": "livox0000000001"}
    
    livox_ros_driver2 = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            xfer_format,
            multi_topic,
            data_src,
            publish_freq,
            output_data_type,
            frame_id,
            lvx_file_path,
            user_config_path,
            cmdline_bd_code
        ]
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", 
            PathJoinSubstitution([FindPackageShare("livox_ros_driver2"), LaunchConfiguration("rviz_config_file")
        ])],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    return LaunchDescription([
        xfer_format_arg,
        multi_topic_arg,
        publish_freq_arg,
        config_file_arg,
        rviz_arg,
        rviz_config_file_arg,
        livox_ros_driver2, 
        rviz2
    ])
