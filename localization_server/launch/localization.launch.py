"""
Localization Server + RViz2 (single argument: map_file)

Usage:
  ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml
  ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml
  ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_sim.yaml
  ros2 launch localization_server localization.launch.py map_file:=warehouse_map_keepout_real.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    # --- Declare Launch Arguments ---
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="warehouse_map_sim.yaml",
        description="Map YAML filename located in the package's config/ folder"
    )
    map_file = LaunchConfiguration('map_file')

    # True for the sim map, False otherwise (drives use_sim_time & AMCL config choice)
    is_sim_expr = PythonExpression(["'sim' in '", map_file, "'"])
    use_sim_time = ParameterValue(is_sim_expr, value_type=bool)
    # True if the map file indicates keepout support
    has_keepout = PythonExpression(["'keepout' in '", map_file,"'"])

    # --- Dynamic Paths ---
    map_yaml = PathJoinSubstitution([
        FindPackageShare("map_server"), 
        "config", 
        map_file
    ])
    amcl_yaml = PathJoinSubstitution([
        FindPackageShare("localization_server"), 
        "config", 
        PythonExpression([
            "'amcl_config_sim.yaml' if (", is_sim_expr, ") else 'amcl_config_real.yaml'"
        ])
    ])
    filters_yaml = PathJoinSubstitution([
        FindPackageShare("localization_server"), 
        "config", 
        'filters.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("localization_server"),
        "rviz",
            PythonExpression([
            "'localizer_sim.rviz' if (", is_sim_expr, ") else 'localizer_real.rviz'"
        ])
    ])

    # --- Logs ---
    log_map = LogInfo(msg=["Map YAML: ", map_yaml])

    return LaunchDescription([
        map_file_arg,
        log_map,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_yaml}]
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                {"use_sim_time": use_sim_time},
                amcl_yaml]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml, {'yaml_filename': map_yaml}],
            condition=IfCondition(has_keepout)
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml],
            condition=IfCondition(has_keepout)
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_filters',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
            condition=IfCondition(has_keepout)
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=[
                        "-d",
                        rviz_config,
                    ],
                ),
            ]
        )
    ])