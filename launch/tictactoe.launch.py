from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    cam2image = Node(
        name="cam2image",
        package="image_tools",
        executable="cam2image",
        parameters=[
            {"frequency": 20.0},
            {"reliability": "reliable"}
        ]
    )

    turtlebot_bringup_share_dir = get_package_share_directory("turtlebot3_bringup")
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_bringup_share_dir, 'launch/robot.launch.py')
        )
    )

    tictactoe = Node(
        name="tictactoe",
        package="projekt",
        executable="image_processor"
    )

    goto_pose = Node(
        name="go_to_position",
        package="projekt",
        executable="navigate_to_pose"
    )

    return LaunchDescription([
        robot_bringup,
        cam2image,
        tictactoe,
#        goto_pose
    ])
