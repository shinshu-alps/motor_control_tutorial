import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    control_commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("motor_control"), "launch"),
                "/control_commander.launch.py",
            ]
        )
    )
    motor_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("motor_control"), "launch"),
                "/motor.launch.py",
            ]
        )
    )
    tools = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("motor_control"), "launch"),
                "/tools.launch.py",
            ]
        )
    )
    visualize = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("motor_control"), "launch"),
                "/visualize.launch.py",
            ]
        )
    )

    return launch.LaunchDescription(
        [
            control_commander,
            motor_nodes,
            tools,
            visualize,
        ]
    )
