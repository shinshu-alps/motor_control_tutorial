import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パラメータの読み込み
    param_config = os.path.join(
        get_package_share_directory("motor_control"), "param_cfg", "angle_velocity_control.yaml"
    )

    angle_velocity_control_node = launch_ros.actions.Node(
        package="motor_control", executable="angle_velocity_control", parameters=[param_config]
    )
    motor_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("motor_control"), "launch"),
                "/sim_motor.launch.py",
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

    return launch.LaunchDescription(
        [
            angle_velocity_control_node,
            motor_nodes,
            tools,
        ]
    )