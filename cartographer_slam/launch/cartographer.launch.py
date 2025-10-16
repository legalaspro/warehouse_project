"""
Cartographer + RViz2 launch (sim/real toggle).

Usage:
  ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=True
  ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=False
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True', 
        description='Whether simultaion or real robot used'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Dynamic paths selection ---
    cartographer_share_pkg = FindPackageShare("cartographer_slam")
    cartographer_config_dir = PathJoinSubstitution([cartographer_share_pkg, 'config'])

    # Select sim or real rviz configs
    configuration_basename = PythonExpression([
        "'cartographer_sim.lua' if '", use_sim_time, "' == 'True' else 'cartographer_real.lua'"
    ])
    rviz_config_basename = PythonExpression([
        "'mapping_sim.rviz' if '", use_sim_time, "' == 'True' else 'mapping_real.rviz'"
    ])
    rviz_config = PathJoinSubstitution([cartographer_share_pkg, "rviz", rviz_config_basename])
    
    # ---- Logs ---
    log_config_choice = LogInfo(
        msg=["Selected cartographer config: ", configuration_basename]
    )
    log_time_mode = LogInfo(
        msg=["use_sim_time = ", use_sim_time]
    )

    # RViz2 Node with Delay
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config]
    )

    # Delay RViz by 3 seconds 
    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([ 
        use_sim_time_arg,
        log_config_choice,
        log_time_mode,
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        delayed_rviz
    ]) 