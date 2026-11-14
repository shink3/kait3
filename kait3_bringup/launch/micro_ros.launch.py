from launch import LaunchDescription
from launch_ros.actions import Node
from typing import Final

DEVICE_PATH: Final[str] = "/dev/ttyACM0"
BAUD_RATTE: Final[str]  = "115200"
LOG_LEVEL: Final[str]   = "4"

def generate_launch_description() -> LaunchDescription:
    
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=[
            "serial",
            "-D", DEVICE_PATH,
            "-b", BAUD_RATTE,
            "-v", LOG_LEVEL
        ],
        output="log"
    )


    return LaunchDescription([micro_ros_agent])



