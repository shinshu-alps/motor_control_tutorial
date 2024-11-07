import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            # launch_ros.actions.Node(
            #     package="motor_control",
            #     executable="can_ros_bridge",
            # ),
            launch_ros.actions.Node(
                package="motor_control",
                executable="angle_control",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="rig_reconfigure",
                executable="rig_reconfigure",
            ),
        ]
    )
