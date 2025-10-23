#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import std_msgs.msg  # For lift command (Bool)
from geometry_msgs.msg import Polygon, Point32
from std_srvs.srv import Trigger  # Simple success/failure; or use custom srv
from std_msgs.msg import String
import math
import time
import sys

class ShelfHandlerNode(Node):
    def __init__(self):
        super().__init__('shelf_handler_node')
        
        # Publishers for lift and footprint
        self.liftup_pub = self.create_publisher(String, '/elevator_up', 10)
        self.liftdown_pub = self.create_publisher(String, '/elevator_down', 10)
        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 1)
        self.local_footprint_pub = self.create_publisher(Polygon, '/local_costmap/footprint', 1)
        
        # Define constants
        # self.FOOTPRINT_SHELF_UP = [(0.35,0.3),(0.35,-0.3),(-0.35,-0.3),(-0.35,0.3)]
        self.FOOTPRINT_SHELF_UP = [(0.45, 0.4), (0.45, -0.4), (-0.45, -0.4), (-0.45, 0.4)]
        self.FOOTPRINT_ROBOT_RADIUS = 0.25  # Assuming a default radius; adjust as needed


        self.get_logger().info('Shelf Handler Node ready (lift and footprint only)')

    def lift_up(self):
        msg = String()
        for _ in range(10):
            self.liftup_pub.publish(msg)
            time.sleep(0.1)
        self.update_footprint(attached=True)
        self.get_logger().info('Published lift up command and updated footprint')

    def lift_down(self):
        msg = String()
        for _ in range(10):
            self.liftdown_pub.publish(msg)
            time.sleep(0.1)
        self.update_footprint(attached=False)
        self.get_logger().info('Published lift down command and updated footprint')

    def update_footprint(self, attached: bool = False):
        """Update robot footprint polygon for costmaps."""
        footprint = Polygon()
        if attached:
            for p in self.FOOTPRINT_SHELF_UP:
                footprint.points.append(Point32(x=p[0], y=p[1], z=0.0))
        else:
            n = 36
            r = self.FOOTPRINT_ROBOT_RADIUS
            for i in range(n):
                theta = 2 * math.pi * i / n
                footprint.points.append(Point32(x=r * math.cos(theta), y=r*math.sin(theta), z=0.0))
        
        self.global_footprint_pub.publish(footprint)
        self.local_footprint_pub.publish(footprint)

def main(args=None):
    rclpy.init(args=args)
    
    node = ShelfHandlerNode()
    
    if len(sys.argv) > 1 and sys.argv[1].lower() == 'up':
        node.lift_up()
    else:
        node.lift_down()  # Default to down if no arg or other
    
    # Spin briefly to allow publish
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()