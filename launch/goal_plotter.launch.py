import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    map_name = "2005375_full_map.yaml"
    # goal file to save goal poses to.
    save_goal_file = "goal_test.json"
    # goal file to load poses from
    load_goal_file = "goal_test.json"
    autostart = True
    use_sim_time = False

    map_yaml_file = os.path.join(
        get_package_share_directory("warehousebot_navigation"), "maps", map_name
    )
    params_file = os.path.join(
        get_package_share_directory("goal_plotter"), "param", "map_server_config.yaml"
    )
    rviz_config_dir = os.path.join(
        get_package_share_directory("goal_plotter"), "rviz", "default_plot.rviz"
    )

    save_file_path = os.path.join(
        get_package_share_directory("goal_plotter"), "goal_json", save_goal_file
    )

    load_file_path = os.path.join(
        get_package_share_directory("goal_plotter"), "goal_json", load_goal_file
    )

    param_substitutions = {"yaml_filename": map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    run_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[configured_params],
    )

    run_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"node_names": ["map_server"]}, {"autostart": autostart}],
    )

    # run rviz2 with settings
    run_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    run_main_plot = Node(
        package="goal_plotter",
        executable="main_plot",
        name="main_plot",
        parameters=[
            {"save_file_path": save_file_path},
            {"load_file_path": load_file_path},
        ],
        prefix=["xterm -e gdb -ex run --args"],
        output="screen",
    )

    run_get_goal = Node(
        package="goal_plotter",
        executable="get_goal",
        name="get_goal",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(run_lifecycle_manager)
    ld.add_action(run_map_server)
    ld.add_action(run_rviz2)
    ld.add_action(run_main_plot)
    ld.add_action(run_get_goal)

    return ld
