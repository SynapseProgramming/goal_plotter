import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # goal file to load poses from
    load_goal_file = "goal_test.json"

    load_file_path = os.path.join(
        get_package_share_directory("goal_plotter"), "goal_json", load_goal_file
    )
    run_goal_manager = Node(
        package="goal_plotter",
        executable="lv_goal_manager.py",
        name="goal_manager",
        emulate_tty=True,
        parameters=[
            {"load_file_path": load_file_path, "robot_radius": 0.3, "reject_cost": 254}
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(run_goal_manager)

    return ld
