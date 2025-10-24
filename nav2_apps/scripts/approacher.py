#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from geometry_msgs.msg import Polygon, Point32, Point, PointStamped, PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import LaserScan
import math
import sys
import time
import threading
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_geometry_msgs import do_transform_point
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.logging import LoggingSeverity

class ShelfHandlerNode(Node):
    def __init__(self):
        super().__init__('shelf_handler_node')

        # Set the Logger Level to DEBUG
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.get_logger().debug('✅ Logger set to DEBUG level inside the node code.')

        ## Parameters (adjust these as needed for your setup)
        self.leg_intensity_threshold = 4000  # Intensity threshold for reflective markers on legs
        self.min_points_per_leg = 5  # Minimum points to consider a cluster a leg
        self.MAP_FRAME = 'map'  # Map frame name
        self.CART_FRAME = 'cart_frame'  # Child frame for cart
        self.base_footprint_frame = 'robot_base_footprint'  # Robot base frame
        self.odom_frame = 'robot_odom'  # Odom frame
        self.cmd_pub_topic = '/cmd_vel'  # Command velocity topic
        
        self.kp_dist = 0.5  # Proportional gain for distance
        self.kp_yaw = 2.0  # Proportional gain for yaw
        self.v_min = 0.1  # Min linear speed (m/s)
        self.v_max = 0.2  # Max linear speed (m/s)
        self.w_max = 1.0  # Max angular speed (rad/s)
        self.stop_dist = 0.175  # Stopping distance tolerance (m)
        self.yaw_gate = 0.15  # Yaw error threshold to allow linear motion
        self.target_offset = 0.3  # Additional advance distance in phase 2 (m)
        self.rotate_yaw_tol = 0.0175  # Rotation tolerance (rad) 3 degree
        self.rotate_min_vel = 0.15 # Rotation min velocity

        # Callback group for reentrant callbacks
        reentrant_group = ReentrantCallbackGroup()

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10, callback_group=reentrant_group
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_pub_topic, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Rate and state
        self.rate = self.create_rate(10.0)  # 10 Hz
        self.log_rate = self.create_rate(1.0)  # 1 Hz for info/debug logs
        self.latest_scan = None

        self.get_logger().info('Shelf Handler Node ready for approach testing')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def position_under_shelf(self):
        """Position the robot under the shelf: detect legs, publish TF, move under, rotate."""
        if self.latest_scan is None:
            self.get_logger().error("No LaserScan received yet.")
            return False
       
        legs = self.detect_shelf_legs(self.latest_scan)
        if len(legs) != 2:
            self.get_logger().error(f"Detected {len(legs)} shelf legs, while need 2 shelf legs")
            return False

        self.get_logger().info(f"Identified two legs with centers: ({legs[0][0]:.3f}, {legs[0][1]:.3f}), ({legs[1][0]:.3f}, {legs[1][1]:.3f})")
        
        if not self.publish_cart_frame(legs, 
            self.latest_scan.header.frame_id, 
            self.latest_scan.header.stamp):
            self.get_logger().error("Failed to publish cart frame static TF!")
            return False
        
        if not self.move_under_shelf():
            self.get_logger().error("Failed to move under shelf!")
            return False
        
        self.rotate_degrees(180)
    
        self.get_logger().info("Position under shelf successfully!")
        return True

    def detect_shelf_legs(self, scan):
        """Detect shelf legs using intensity clustering from LaserScan."""
        leg_centers = []
        intensities = scan.intensities # Intensity threshold for reflective markers
       
        sx = 0.0
        sy = 0.0
        count = 0
        current_angle = scan.angle_min
        for i, intensity in enumerate(intensities):
            r = scan.ranges[i]
            
            # validate range: break cluster and skip if invalid
            if math.isinf(r) or math.isnan(r):
                if count > self.min_points_per_leg:
                    self.get_logger().info(f"Found cluster with {count} points")
                    leg_centers.append((sx / count, sy / count))
                    sx = sy = count = 0.0
                current_angle += scan.angle_increment
                continue
            
            if intensity >= self.leg_intensity_threshold:
                sx += r * math.cos(current_angle)
                sy += r * math.sin(current_angle)
                count += 1
            else:
                if count > self.min_points_per_leg:
                    self.get_logger().info(f"Found cluster with {count} points")
                    leg_centers.append((sx/count, sy/count))
                    sx = sy = count = 0.0
            current_angle += scan.angle_increment

        # Handle any leftover cluster
        if count > self.min_points_per_leg:
            self.get_logger().info(f"Found cluster with {count} points")
            leg_centers.append((sx / count, sy / count))
        
        return leg_centers

    def publish_cart_frame(self, legs, laser_frame, timestamp):
        """Publish static TF for cart_frame based on detected legs."""
        if len(legs) != 2:
            self.get_logger().warn(f"Expected exactly 2 legs, but found {len(legs)}. Skipping TF publication.")
            return False

        # Cart center in laser frame
        cart_frame_laser = PointStamped()
        cart_frame_laser.header.frame_id = laser_frame
        cart_frame_laser.header.stamp = timestamp 
        cart_frame_laser.point.x = (legs[0][0] + legs[1][0]) / 2
        cart_frame_laser.point.y = (legs[0][1] + legs[1][1]) / 2
        cart_frame_laser.point.z = 0.0
        
        # Leg points in laser frame
        shelf_leg1_laser = PointStamped(header=cart_frame_laser.header, point=Point(x=legs[0][0], y=legs[0][1], z=0.0))
        shelf_leg2_laser = PointStamped(header=cart_frame_laser.header, point=Point(x=legs[1][0], y=legs[1][1], z=0.0))

        # time.sleep(1)

        try:
            # Use current time for fresh transform
            transform = self.tf_buffer.lookup_transform(self.MAP_FRAME, laser_frame, rclpy.time.Time(), timeout=Duration(seconds=0.5))

            shelf_leg1_map = do_transform_point(shelf_leg1_laser, transform)
            shelf_leg2_map = do_transform_point(shelf_leg2_laser, transform)
            cart_frame_map = do_transform_point(cart_frame_laser, transform)

            # Yaw calculation: along leg line, then perpendicular inward
            dy = shelf_leg2_map.point.y - shelf_leg1_map.point.y
            dx = shelf_leg2_map.point.x - shelf_leg1_map.point.x
            leg_yaw = math.atan2(dy, dx)
            # Face "into" the cart: perpendicular to leg line.
            # +pi/2 for one direction (e.g., inward)
            face_yaw = leg_yaw + math.pi / 2
            
            # Publish TF
            chart_frame_tf = TransformStamped()
            chart_frame_tf.header.stamp = self.get_clock().now().to_msg()
            chart_frame_tf.header.frame_id = self.MAP_FRAME
            chart_frame_tf.child_frame_id = self.CART_FRAME
            chart_frame_tf.transform.translation.x = cart_frame_map.point.x
            chart_frame_tf.transform.translation.y = cart_frame_map.point.y
            chart_frame_tf.transform.translation.z = cart_frame_map.point.z
            quat = quaternion_from_euler(0, 0, face_yaw)
            chart_frame_tf.transform.rotation.x = quat[0]
            chart_frame_tf.transform.rotation.y = quat[1]
            chart_frame_tf.transform.rotation.z = quat[2]
            chart_frame_tf.transform.rotation.w = quat[3]

            self.tf_static_broadcaster.sendTransform(chart_frame_tf)
            self.rate.sleep()   # Brief wait for TF propagation
            self.get_logger().info(f"Successfully published {self.CART_FRAME} TF.")
            return True
        except TransformException as ex:
            self.get_logger().error(f"Failed to publish  {self.CART_FRAME} TF: {ex}")
            return False
    
    def move_under_shelf(self):
        """Move under the shelf using TF-based control (phase 1: approach, phase 2: advance)."""
        phase = 1 # Start with approach phase

        while rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                     self.base_footprint_frame, self.CART_FRAME, rclpy.time.Time(), Duration(seconds=0.1)
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed: {ex}")
                return False

            dx = tf.transform.translation.x
            dy = tf.transform.translation.y
            error_distance = math.hypot(dx, dy)

            if phase == 1:
                # Phase 1: Approach cart_frame position with heading-based yaw
                error_yaw = math.atan2(dy, dx)
                self.get_logger().debug(f"Phase 1 - Distance: {error_distance:.3f}, Yaw: {error_yaw:.3f}, dx={dx:.3f}, dy={dy:.3f}")
                if dx >= 0.0 and abs(dx) <= self.stop_dist and abs(dy) <= self.stop_dist:
                    self.get_logger().info(f"Reached cart frame ({error_distance:.3f} m). Transitioning to phase 2.")
                    phase = 2  
                    continue  # Fresh TF for phase 2
            
            if phase == 2:
                # Phase 2: Advance additional 40cm while aligning orientation
                quat = [
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                ]
                _, _, error_yaw = euler_from_quaternion(quat)
                self.get_logger().debug(f"Phase 2 - Distance: {error_distance:.3f}")
                if error_distance >= self.target_offset:
                    self.get_logger().info(f"Advance complete ({error_distance:.3f} m).")
                    break
            
            # Compute and publish command
            cmd = Twist()
            cmd.linear.x = max(self.v_min, min(self.kp_dist * error_distance, self.v_max))
            cmd.angular.z = max(-self.w_max, min(self.kp_yaw * error_yaw, self.w_max))

            if abs(error_yaw) > self.yaw_gate:
                cmd.linear.x = 0.0  # Turn in place,
            
            self.cmd_pub.publish(cmd)
            self.get_logger().debug(f"Published cmd: linear.x={cmd.linear.x:.3f}, angular.z={cmd.angular.z:.3f}")
            self.rate.sleep()
        
        # Always stop at end
        self.cmd_pub.publish(Twist())
        return True

    def rotate_degrees(self, degrees: float = 180.0):
        """Rotate the robot by specified degrees using odom TF."""
        initial_yaw = None  
        target_yaw = None

        while rclpy.ok():
            try:
                tf_odom_to_robot = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_footprint_frame, rclpy.time.Time(), Duration(seconds=0.1)
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF lookup failed: {ex}")
                return False

            quat = [
                tf_odom_to_robot.transform.rotation.x,
                tf_odom_to_robot.transform.rotation.y,
                tf_odom_to_robot.transform.rotation.z,
                tf_odom_to_robot.transform.rotation.w,
            ]
            _, _, current_yaw = euler_from_quaternion(quat)

            if initial_yaw is None:  # Set targets on first entry
                initial_yaw = current_yaw
                target_yaw = initial_yaw + math.radians(degrees)

            # Normalize error to [-pi, pi]
            error_yaw = target_yaw - current_yaw
            error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi
            self.get_logger().debug(f"Rotation error: {error_yaw:.3f}")
            
            if abs(error_yaw) <= self.rotate_yaw_tol:
                self.get_logger().info(f"{degrees}-degree rotation complete.")
                break

            # Compute and publish command
            cmd = Twist()
            cmd.linear.x = 0.0
            angular_z = self.kp_yaw * error_yaw
            if abs(angular_z) < self.rotate_min_vel:
                angular_z = math.copysign(self.rotate_min_vel, error_yaw)
            cmd.angular.z = max(-self.w_max, min(angular_z, self.w_max))
            self.cmd_pub.publish(cmd)
            self.rate.sleep()
        
        # Stop
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    
    node = ShelfHandlerNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        time.sleep(2.0)  # Adjust based on your setup
        if len(sys.argv) > 1:
            node.rotate_degrees()
        elif len(sys.argv) == 1:
            # Wait briefly for initial scan data if needed
            node.position_under_shelf()
        
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()