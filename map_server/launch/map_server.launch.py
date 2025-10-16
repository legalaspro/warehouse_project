"""
Map Server + RViz2 (single argument: map_file)

Usage:
  ros2 launch cartographer_slam map_server.launch.py map_file:=warehouse_map_sim.yaml
  ros2 launch cartographer_slam map_server.launch.py map_file:=warehouse_map_real.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # --- Declare Launch Arguments ---
    map_file_arg = DeclareLaunchArgument(
        "map_file",
        default_value="warehouse_map_sim.yaml",
        description="Map YAML filename located in the package's maps/ folder"
    )
    map_file = LaunchConfiguration('map_file')
    
    # --- Dynamic Paths ---
    pkg_share   = FindPackageShare("map_server")
    map_yaml    = PathJoinSubstitution([pkg_share, "config", map_file])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "map_display.rviz"])

    # --- Logs ---
    log_map = LogInfo(msg=["Map YAML: ", map_yaml])

    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_yaml}]
    )

    # Lifecycle Manager Node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # RViz2 Node with Delay
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config]
    )

    # Delay RViz by 3 seconds 
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        map_file_arg,
        log_map,
        map_server_node,
        lifecycle_manager_node,
        delayed_rviz 
    ])