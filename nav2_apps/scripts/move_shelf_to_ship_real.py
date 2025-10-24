#! /usr/bin/env python3
import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from warehouse_orchestrator import WarehouseOrchestrator  
from rclpy.logging import LoggingSeverity
import argparse
import sys

def main(args=None):

    parser = argparse.ArgumentParser(description='Warehouse Orchestrator Node')
    parser.add_argument('--debug', action='store_true', help='Enable DEBUG logging')
    parsed_args = parser.parse_args(args=args[1:] if args else sys.argv[1:])

    rclpy.init()
    node = WarehouseOrchestrator()

    # Set log level based on flag
    log_level = LoggingSeverity.DEBUG if parsed_args.debug else LoggingSeverity.INFO
    node.get_logger().set_level(log_level)
    node.get_logger().info(f'Set log level to {"DEBUG" if parsed_args.debug else "INFO"}')

    # Set real-specific params
    params = [
        Parameter(name='odom_frame', value='robot_odom'),
        Parameter(name='base_footprint_frame', value='robot_base_footprint'),
        Parameter(name='cmd_pub_topic', value='/cmd_vel'),
        Parameter(name='lift_publish_count', value=5),
        Parameter(name='waypoints.init_position', value=[0.0, 0.0, 0.0]),
        Parameter(name='waypoints.loading_position', value=[4.3, -0.3, math.radians(-90)]),
        Parameter(name='waypoints.preshipping_position', value=[2.0, -0.1, math.radians(180)]),
        Parameter(name='waypoints.shipping_position', value=[2.0, 1.1, math.radians(90)]),
        # Attachment/movement configs
        Parameter(name='kp_dist', value=0.5),
        Parameter(name='kp_yaw', value=2.0),
        Parameter(name='v_min', value=0.1),
        Parameter(name='v_max', value=0.2),
        Parameter(name='w_max', value=1.0),
        Parameter(name='stop_dist', value=0.175),
        Parameter(name='target_offset', value=0.3),
        Parameter(name='yaw_gate', value=0.15), # 8.5 degrees gate (test lower)
        Parameter(name='rotate_yaw_tol', value=0.0175), 
        Parameter(name='rotate_min_vel', value=0.15), # min requiered speed for robot
        # Leg detection configs
        Parameter(name='leg_intensity_threshold', value=4000),
        Parameter(name='min_points_per_leg', value=5),
        # Footprint configs
        Parameter(name='footprint_shelf_up', value=[0.45, 0.4, 0.45, -0.4, -0.45, -0.4, -0.45, 0.4]),
        Parameter(name='footprint_robot_radius', value=0.25),
    ]
    node.set_parameters(params)
    node.get_logger().set_level(LoggingSeverity.DEBUG)

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
    main(sys.argv)