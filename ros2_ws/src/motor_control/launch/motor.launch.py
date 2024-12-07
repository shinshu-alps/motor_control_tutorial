import launch
import launch_ros.actions
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_argument = DeclareLaunchArgument("sim", default_value="true")
    sim = LaunchConfiguration("sim")

    return launch.LaunchDescription(
        [
            sim_argument,
            launch_ros.actions.Node(
                package="motor_control",
                executable="can_ros_bridge",
                condition=UnlessCondition(sim),
            ),
            launch_ros.actions.Node(
                package="motor_control",
                executable="sim_motor",
                condition=IfCondition(sim),
            ),
        ],
    )
