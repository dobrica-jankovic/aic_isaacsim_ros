"""Launch port perception plus the insertion controller against a running sim.

    ros2 launch aic_insertion insertion.launch.py            # full pipeline
    ros2 launch aic_insertion insertion.launch.py control:=false   # perception only
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control = LaunchConfiguration("control")
    debug = LaunchConfiguration("debug_overlay")
    return LaunchDescription(
        [
            DeclareLaunchArgument("control", default_value="true"),
            DeclareLaunchArgument("debug_overlay", default_value="true"),
            Node(
                package="aic_insertion",
                executable="perception_node",
                output="screen",
                parameters=[{"use_sim_time": True, "debug_overlay": debug}],
            ),
            Node(
                package="aic_insertion",
                executable="insertion_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(control),
            ),
        ]
    )
