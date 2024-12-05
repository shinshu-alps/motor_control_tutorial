import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パラメータの読み込み
    param_config = os.path.join(
        get_package_share_directory("motor_control"),
        "param_cfg",
        "control_commander.yaml",
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="motor_control",
                executable="control_commander",
                parameters=[param_config],
            ),
            launch_ros.actions.Node(
                package="joy",
                executable="joy_node",
            ),
        ]
    )
