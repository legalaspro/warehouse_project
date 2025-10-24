#! /usr/bin/env python3
import math
from threading import Event

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point, Point32, PointStamped, Polygon, PoseStamped, TransformStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, SetParametersResult

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class WarehouseOrchestrator(Node):
    """
    Drives to waypoints and picks/places the shelf.
    This node orchestrates navigation, shelf detection, attachment, lifting, and parameter updates for a warehouse robot.
    """
    
    # Frames
    CART_FRAME = "cart_frame"
    MAP_FRAME = "map"

    def __init__(self):
        super().__init__('warehouse_orchestrator')

        # Declare parameters (with defaults for sim)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_footprint_frame', 'robot_base_footprint')
        self.declare_parameter('cmd_pub_topic', '/diffbot_base_controller/cmd_vel_unstamped')
        self.declare_parameter('lift_publish_count', 3)
        self.declare_parameter('waypoints.init_position', [0.0, 0.0, 0.0])
        self.declare_parameter('waypoints.loading_position', [5.6, 0.0, math.radians(-90)])
        self.declare_parameter('waypoints.preshipping_position', [2.45, 0.0, math.radians(180)])
        self.declare_parameter('waypoints.shipping_position', [2.45, 1.1, math.radians(90)])
        # Attachment/movement configs
        self.declare_parameter('kp_dist', 0.5)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('v_min', 0.1)
        self.declare_parameter('v_max', 0.25)  # min and max linear velocity
        self.declare_parameter('w_max', 1.0)  # max angular velocity
        self.declare_parameter('stop_dist', 0.01)  # stop distance
        self.declare_parameter('target_offset', 0.4)   # Advance distance
        self.declare_parameter('yaw_gate', 0.5)  # ~30deg: rotate-in-place when misaligned
        self.declare_parameter('rotate_yaw_tol', 0.05) # Tolerance about 3 degrees
        self.declare_parameter('rotate_min_vel', 0.1) # Minimum angular velocity to rotate
        # Leg detection configs
        self.declare_parameter('leg_intensity_threshold', 2000)
        self.declare_parameter('min_points_per_leg', 3)
        # Footprint configs
        self.declare_parameter('footprint_shelf_up', [0.5, 0.45, 0.5, -0.45, -0.5, -0.45, -0.5, 0.45])  # Flattened list: x1,y1,x2,y2,...
        self.declare_parameter('footprint_robot_radius', 0.25)
        
        # Fetch params
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_footprint_frame = self.get_parameter('base_footprint_frame').value
        self.cmd_pub_topic = self.get_parameter('cmd_pub_topic').value
        self.lift_publish_count = self.get_parameter('lift_publish_count').value
        self.waypoints = {
            'init_position': self.get_parameter('waypoints.init_position').value,
            'loading_position': self.get_parameter('waypoints.loading_position').value,
            'preshipping_position': self.get_parameter('waypoints.preshipping_position').value,
            'shipping_position': self.get_parameter('waypoints.shipping_position').value,
        }
        self.kp_dist = self.get_parameter('kp_dist').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.stop_dist = self.get_parameter('stop_dist').value
        self.target_offset = self.get_parameter('target_offset').value
        self.yaw_gate = self.get_parameter('yaw_gate').value
        self.rotate_yaw_tol = self.get_parameter('rotate_yaw_tol').value
        self.rotate_min_vel = self.get_parameter('rotate_min_vel').value
        self.leg_intensity_threshold = self.get_parameter('leg_intensity_threshold').value
        self.min_points_per_leg = self.get_parameter('min_points_per_leg').value
        # Fetch and reshape footprints
        flat_shelf_up = self.get_parameter('footprint_shelf_up').value
        self.footprint_shelf_up = [(flat_shelf_up[i], flat_shelf_up[i+1]) for i in range(0, len(flat_shelf_up), 2)]
        self.footprint_robot_radius = self.get_parameter('footprint_robot_radius').value

        # Register the parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Navigator setup
        self.navigator_ = BasicNavigator()

        # Callback group for reentrancy 
        reentrant_group = ReentrantCallbackGroup()

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10, callback_group=reentrant_group
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_pub_topic, 10)
        self.liftup_pub = self.create_publisher(String, '/elevator_up', 10)
        self.liftdown_pub = self.create_publisher(String, '/elevator_down', 10)
        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 1)
        self.local_footprint_pub = self.create_publisher(Polygon, '/local_costmap/footprint', 1)

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Rate and state
        self.rate = self.create_rate(10.0) # 10 Hz
        self.log_rate = self.create_rate(1.0)  # 1 Hz for info/debug logs
        self.latest_scan = None

        # Set initial pose and wait for Nav2
        self.set_initial_pose()
        self.navigator_.waitUntilNav2Active() 

        # Startup timer
        self._startup_timer = self.create_timer(0.5, self.start_once, callback_group=reentrant_group)
        self.get_logger().info("Warehouse Orchestrator Node is Ready!")

    def parameter_callback(self, params):
        """
        Callback function for setting parameters at runtime.
        Directly assigns new parameter values without validation or type-checking.
        """
        
        for param in params:
            # Simple String/Topic Parameters
            if param.name == 'odom_frame':
                self.odom_frame = param.value
            elif param.name == 'base_footprint_frame':
                self.base_footprint_frame = param.value
            elif param.name == 'cmd_pub_topic':
                self.cmd_pub_topic = param.value
                # Destroy the old publisher if it exists
                if hasattr(self, 'cmd_pub') and self.cmd_pub is not None:
                    self.destroy_publisher(self.cmd_pub)
                self.cmd_pub = self.create_publisher(Twist, self.cmd_pub_topic, 10)
                self.get_logger().info(f"Cmd Velocity publisher moved to: {self.cmd_pub_topic}")
            
            # Integer/Count parameters
            elif param.name == 'lift_publish_count':
                self.lift_publish_count = param.value
            elif param.name == 'min_points_per_leg':
                self.min_points_per_leg = param.value
            
            # Floating-point/Control parameters
            elif param.name == 'kp_dist':
                self.kp_dist = param.value
            elif param.name == 'kp_yaw':
                self.kp_yaw = param.value
            elif param.name == 'v_min':
                self.v_min = param.value
            elif param.name == 'v_max':
                self.v_max = param.value
            elif param.name == 'w_max':
                self.w_max = param.value
            elif param.name == 'stop_dist':
                self.stop_dist = param.value
            elif param.name == 'target_offset':
                self.target_offset = param.value
            elif param.name == 'yaw_gate':
                self.yaw_gate = param.value
            elif param.name == 'rotate_yaw_tol':
                self.rotate_yaw_tol = param.value
            elif param.name == 'rotate_min_vel':
                self.rotate_min_vel = param.value
            elif param.name == 'leg_intensity_threshold':
                self.leg_intensity_threshold = param.value
            elif param.name == 'footprint_robot_radius':
                self.footprint_robot_radius = param.value
            
            # List-based parameters (Waypoints and Footprint)
            elif param.name.startswith('waypoints.'):
                # Assumes value is a list of 3 numbers
                waypoint_key = param.name.split('.')[1]
                self.waypoints[waypoint_key] = param.value
            elif param.name == 'footprint_shelf_up':
                # Assumes value is a flat list of coordinates and reshapes it
                flat_shelf_up = param.value
                self.footprint_shelf_up = [(flat_shelf_up[i], flat_shelf_up[i+1]) for i in range(0, len(flat_shelf_up), 2)]

        return SetParametersResult(successful=True)


    def set_initial_pose(self):
        """Set the initial pose from waypoints."""
        x, y, yaw = self.waypoints['init_position']
        self.initial_pose_ = self.make_pose(x, y, yaw, self.MAP_FRAME)
        self.initial_pose_.header.stamp = self.get_clock().now().to_msg()
        self.navigator_.setInitialPose(self.initial_pose_)
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def start_once(self):
        """Run the sequence once after startup."""
        if hasattr(self, "_startup_timer") and self._startup_timer is not None:
            self._startup_timer.cancel()
            self.destroy_timer(self._startup_timer)
            self._startup_timer = None
                
        self.run_sequence()

    def run_sequence(self):
        """Execute the full warehouse sequence."""
        self.get_logger().info("Executing the sequence!")
        self.get_logger().info("Phase 1: Go to loading and attach")
        if not self.goto('loading_position'): return
        if not self.position_under_shelf(): return
        self.lift_shelf(raise_up=True)
        
        self.get_logger().info("Phase 2: Transport to shipping")
        if not self.goto('preshipping_position'): return
        if not self.goto('shipping_position'): return
        self.lift_shelf(raise_up=False)
        self.rotate_degrees(180)

        self.get_logger().info("Phase 3: Return to init")
        if not self.goto('preshipping_position'): return
        self.goto('init_position')
        self.get_logger().info("Mission Complete!")
        rclpy.shutdown()
   
    def position_under_shelf(self):
        """Position the robot under the shelf: detect legs, publish TF, move under, rotate."""
        self.log_rate.sleep() # add small sleep
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
                    leg_centers.append((sx/count, sy/count))
                    sx = sy = count = 0.0
            current_angle += scan.angle_increment

        # Handle any leftover cluster
        if count > self.min_points_per_leg:
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
            # self.rate.sleep()   # Brief wait for TF propagation
            self.get_logger().info(f"Successfully published {self.CART_FRAME} TF.")
            return True
        except TransformException as ex:
            self.get_logger().error(f"Failed to publish  {self.CART_FRAME} TF: {ex}")
            return False
    
    def move_under_shelf(self):
        """Move under the shelf using TF-based control (phase 1: approach, phase 2: advance)."""
        phase = 1 # Start with approach phase
        self.get_logger().info("Start Move under Shelf")

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
    
    def lift_shelf(self, raise_up: bool = True):
        """Lift or lower the shelf and update footprint/critics."""
        pub = self.liftup_pub if raise_up else self.liftdown_pub
        for _ in range(self.lift_publish_count):
            pub.publish(String())
            self.log_rate.sleep()
        
        self.get_logger().info(f"Lifted the shelf {'up' if raise_up else 'down'}.")
        self.update_footprint(attached=raise_up)
        self.update_controller_critics(attached=True) # we always have obstacle footprint
        # self.update_costmaps(attached=raise_up)
        # self.log_rate.sleep() # add sleep for 1 sec

    def update_footprint(self, attached: bool = False):
        """Update robot footprint polygon for costmaps."""
        footprint = Polygon()
        if attached:
            for p in self.footprint_shelf_up:
                footprint.points.append(Point32(x=p[0], y=p[1], z=0.0))
        else:
            n = 36
            r = self.footprint_robot_radius
            for i in range(n):
                theta = 2 * math.pi * i / n
                footprint.points.append(Point32(x=r * math.cos(theta), y=r*math.sin(theta), z=0.0))
        
        self.global_footprint_pub.publish(footprint)
        self.local_footprint_pub.publish(footprint)
        self.get_logger().info(f"Updated robot footprint (attached: {attached})")

    def goto(self, name: str) -> bool:
        """Navigate to a named waypoint using Nav2."""
        x, y, yaw = self.waypoints[name]
        pose = self.make_pose(x, y, yaw, self.MAP_FRAME)
        pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Navigating to {name} (x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.0f}°)")

        self.navigator_.goToPose(pose)
        
        while not self.navigator_.isTaskComplete():
            feedback = self.navigator_.getFeedback()
            if feedback:
                self.get_logger().debug(f"ETA to {name}: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} sec")
            self.log_rate.sleep()  
        
        result = self.navigator_.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Arrived at {name}")
            return True
        else:
            self.get_logger().error(f"Failed to reach {name}")
            return False
    
    def make_pose(self, x: float, y: float, yaw: float, frame_id: str = "map") -> PoseStamped:
        """Create a PoseStamped message."""
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        _, _, qz, qw = quaternion_from_euler(0, 0, yaw)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg
    
    def update_controller_critics(self, attached: bool = False):
        """Update controller critic scales based on attachment."""
        if attached:
            base_obst = 0.0
            fp_obst = 0.02
        else:
            base_obst = 0.02
            fp_obst = 0.0
        
        params_dict = {
            'FollowPath.BaseObstacle.scale': float(base_obst),
            'FollowPath.ObstacleFootprint.scale': float(fp_obst),
        }

        self.update_parameters('controller_server', params_dict)
    
    def update_costmaps(self, attached: bool = False):
        """Update costmap inflation parameters based on attachment."""
        if attached:
            cost_scaling_factor = 2.0
            inflation_radius = 0.25
        else:
            cost_scaling_factor = 5.0
            inflation_radius = 0.25

        params_dict = {
            'inflation_layer.cost_scaling_factor': float(cost_scaling_factor),
            'inflation_layer.inflation_radius': float(inflation_radius),
        }

        self.update_parameters('local_costmap/local_costmap', params_dict)
        params_dict["inflation_layer.inflation_radius"] =  inflation_radius + 0.1
        self.update_parameters('global_costmap/global_costmap', params_dict)  

    def update_parameters(self, node_name: str, params_dict: dict) -> bool:
        """Update ROS parameters on a node asynchronously."""
        cli = self.create_client(SetParameters, f'/{node_name}/set_parameters')

        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Service {node_name}/set_parameters not available!")
            return

        req = SetParameters.Request()
        req.parameters = []
        for name, value in params_dict.items():
            param = Parameter()
            param.name = name
            param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            req.parameters.append(param)
     
        box = {'resp': None, 'exc': None}
        evt = Event()

        def _on_done(fut):
            try:
                box['resp'] = fut.result()
            except Exception as e:
                box['exc'] = e
            finally:
                evt.set()

        cli.call_async(req).add_done_callback(_on_done)

        if not evt.wait(2.0):
            self.get_logger().warn(f"Timeout setting parameters for {node_name}")
            return False

        if box["exc"] is not None:
            self.get_logger().warn(f"Exception setting parameters for {node_name}: {box['exc']}")
            return False

        self.get_logger().info(f"Set parameters for {node_name}")
        return True
    

def main():
    rclpy.init()
    orchestrator = WarehouseOrchestrator()
    exec = MultiThreadedExecutor(num_threads=4)
    exec.add_node(orchestrator)
    try:
        exec.spin()
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()     