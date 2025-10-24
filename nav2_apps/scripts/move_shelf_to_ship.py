#! /usr/bin/env python3
import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from warehouse_orchestrator import WarehouseOrchestrator

def main():
    rclpy.init()
    node = WarehouseOrchestrator()

    # Simulation Specific Parameters
    params = [
        Parameter(name='odom_frame', value='odom'),
        Parameter(name='base_footprint_frame', value='robot_base_footprint'),
        Parameter(name='cmd_pub_topic', value='/diffbot_base_controller/cmd_vel_unstamped'),
        Parameter(name='lift_publish_count', value=3),
        Parameter(name='waypoints.init_position', value=[0.0, 0.0, 0.0]),
        Parameter(name='waypoints.loading_position', value=[5.6, 0.0, math.radians(-90)]),
        Parameter(name='waypoints.preshipping_position', value=[2.45, 0.0, math.radians(180)]),
        Parameter(name='waypoints.shipping_position', value=[2.45, 1.45, math.radians(90)]),
        # Attachment/movement configs
        Parameter(name='kp_dist', value=0.5),
        Parameter(name='kp_yaw', value=2.0),
        Parameter(name='v_min', value=0.1),
        Parameter(name='v_max', value=0.25),
        Parameter(name='w_max', value=1.0),
        Parameter(name='stop_dist', value=0.05),
        Parameter(name='target_offset', value=0.45),
        Parameter(name='yaw_gate', value=0.2),# 11 degrees gate
        Parameter(name='rotate_yaw_tol', value=0.05),
        Parameter(name='rotate_min_vel', value=0.10), # min requiered speed for robot
        # Leg detection configs
        Parameter(name='leg_intensity_threshold', value=2000),
        Parameter(name='min_points_per_leg', value=5),
        # Footprint configs
        Parameter(name='footprint_shelf_up', value=[0.5, 0.45, 0.5, -0.45, -0.5, -0.45, -0.5, 0.45]),
        Parameter(name='footprint_robot_radius', value=0.25),
    ]
    node.set_parameters(params)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass

if __name__ == '__main__':
    main()