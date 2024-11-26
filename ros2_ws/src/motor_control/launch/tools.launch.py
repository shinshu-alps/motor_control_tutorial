import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パスの取得
    plotjugger_config_path = os.path.join(
        get_package_share_directory("motor_control"),
        "plotjuggler_cfg",
        "for_param_tuning.xml",
    )

    return LaunchDescription(
        [
            # rig_reconfigure
            Node(
                package="rig_reconfigure",
                executable="rig_reconfigure",
            ),
            # PlotJuggler
            Node(
                package="plotjuggler",
                executable="plotjuggler",
                arguments=["-l", plotjugger_config_path],
            ),
        ]
    )
