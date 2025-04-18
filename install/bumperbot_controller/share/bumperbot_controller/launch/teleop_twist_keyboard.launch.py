import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulated time"
    )

    keyboard_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="keyboard_teleop",
        output="screen",
        prefix="xterm -e",  # This line opens a new xterm window to run the node
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ('/cmd_vel', '/raw_cmd_vel')
        ]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            keyboard_teleop,
        ]
    )