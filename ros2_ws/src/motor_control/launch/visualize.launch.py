import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # パスの取得
    rviz_config_path = os.path.join(
        get_package_share_directory("motor_control"),
        "rviz_cfg",
        "visualize.rviz",
    )

    return LaunchDescription(
        [
            # visualize ノード
            Node(
                package="motor_control",
                executable="visualize",
            ),
            # RViz を起動
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )
