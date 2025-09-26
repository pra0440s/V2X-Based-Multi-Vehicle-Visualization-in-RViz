import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='localization_pkg',
            executable='localization_node',
            name='localization_node',
            output='screen'
        )
    ])
