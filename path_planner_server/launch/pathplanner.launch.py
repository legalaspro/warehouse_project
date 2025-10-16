"""
Path Planner + RViz2 launch (sim/real toggle).

Usage:
  ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=True
  ros2 launch path_planner_server pathplanner.launch.py  use_sim_time:=False
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True', 
        description='Whether simultaion or real robot used'
    )
    use_sim_time_cfg = LaunchConfiguration('use_sim_time')

    # Check if True or False
    is_sim = PythonExpression(["'", use_sim_time_cfg, "' == 'True'"])
    use_sim_time = ParameterValue(is_sim, value_type=bool)

    # --- Dynamic paths selection ---
    path_pkg_share = FindPackageShare("path_planner_server")
    path_config_dir = PathJoinSubstitution([path_pkg_share, 'config'])

    # Select sim or real configs
    controller_yaml = PathJoinSubstitution([
        path_config_dir,
        PythonExpression([
            "'controller_sim.yaml' if (", is_sim, ") else 'controller_real.yaml'"
        ])])
    bt_navigator_yaml = PathJoinSubstitution([
        path_config_dir,
        PythonExpression([
            "'bt_navigator_sim.yaml' if (", is_sim, ") else 'bt_navigator_real.yaml'"
        ])])
    planner_yaml = PathJoinSubstitution([
        path_config_dir,
        PythonExpression([
            "'planner_sim.yaml' if (", is_sim, ") else 'planner_real.yaml'"
        ])])
    recovery_yaml = PathJoinSubstitution([
        path_config_dir,
        PythonExpression([
            "'recoveries_sim.yaml' if (", is_sim, ") else 'recoveries_real.yaml'"
        ])])
    cmd_vel_topic = PythonExpression([ "'/diffbot_base_controller/cmd_vel_unstamped' if (", is_sim, ") else '/cmd_vel'"])
    
    rviz_config = PathJoinSubstitution([
        path_pkg_share,
        'rviz',
         PythonExpression([
            "'pathplanning_real.rviz' if (", is_sim, ") else 'pathplanning_real.rviz'"
        ])
    ])

     # --- Logs ---
    log_configs = LogInfo(msg=["Configs: ", controller_yaml, ", ", 
        controller_yaml, ", ", bt_navigator_yaml, ", ", planner_yaml, ", ", recovery_yaml])
    log_cmd_vel = LogInfo(msg=["Cmd Vel Topic: ", cmd_vel_topic])
    log_rviz_config = LogInfo(msg=["Rviz Config: ", rviz_config])

    return LaunchDescription([    
        use_sim_time_arg,
        log_configs,
        log_cmd_vel,
        log_rviz_config,
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[('/cmd_vel', cmd_vel_topic)]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]),
        
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