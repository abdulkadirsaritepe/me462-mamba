import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('minirobots')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'hamster.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'minirobotlarintatlidunyasi.wbt')
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'hamster'},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    obstacle_avoider = Node(
        package='minirobots',
        executable='obstacle_avoider',
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        obstacle_avoider,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])