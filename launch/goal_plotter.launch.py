import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    map_name = "dmro_lab_7jun.yaml"
    autostart = True
    use_sim_time = False

    map_yaml_file = os.path.join(
        get_package_share_directory("simple_navigation"), "map", map_name
    )
    params_file = os.path.join(
        get_package_share_directory("goal_plotter"), "param", "map_server_config.yaml"
    )
    rviz_config_dir = os.path.join(
        get_package_share_directory("goal_plotter"), "rviz", "default_plot.rviz"
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

    ld = LaunchDescription()
    ld.add_action(run_lifecycle_manager)
    ld.add_action(run_map_server)
    ld.add_action(run_rviz2)

    return ld
