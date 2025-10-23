#! /usr/bin/env python3
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped, Point, Twist, Polygon, Point32
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Trigger
from tf2_ros import TransformException 
from tf2_geometry_msgs import do_transform_point
from std_msgs.msg import String
from threading import Event
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from nav_msgs.msg import Odometry


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))

def make_pose(x, y, yaw, frame_id="map"):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    # create quaternion from the euler
    
    _, _, qz, qw = yaw_to_quat(yaw)
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg

# class PutDownService(Node):
#     def __init__(self):
#         super().__init__('put_down_service_node')

#TODO: Try to use only NAV2 for everything, may be cool nope?


class PickUpService(Node):
    def __init__(self):
        super().__init__('pickup_service_node')

        # Define subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Define publibsher
        self.cmd_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.liftup_pub = self.create_publisher(String, '/elevator_up', 10)

        # For the TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.srv = self.create_service(Trigger, '/pickup_shelf', self.pickup_shelf)
        self.rate = self.create_rate(10.0)  # 10 Hz
        self.latest_scan = None

        self.get_logger().info('PickUpService Node is ready!')

    def pickup_shelf(self, request, response):
        
        if self.latest_scan is None:
            self.get_logger().error("No LaserScan recieved yet.")
            response.success = False
            response.message = 'No LaserScan data available yet.'
            return response

        legs = self.detect_shelf_legs(self.latest_scan)
        if len(legs) != 2:
            self.get_logger().error(f"Detected {len(legs)} shelf legs, while need 2 shelf legs")
            response.success = False
            response.message = f"Detected wrong number of legs! ({len(legs)} legs)"
            return response
        
        self.get_logger().info(f"Identified two legs with centers: ({legs[0][0]},{legs[0][1]}), ({legs[1][0]},{legs[1][1]})")
        
        if not self.publish_cart_frame(legs, 
            self.latest_scan.header.frame_id, 
            self.latest_scan.header.stamp):
            response.success = False
            response.message = f"Failed publish cart frame static tf!"
            return response
        
        if not self.run_attach_algorithm():
            response.success = False
            response.message = "Failed attach to the cart!"
            return response
        
        self.get_logger().info("Picked up shelf successfully!")
        response.success = True
        response.message = "Picked up shelf successfully!"
        return response
    
    def detect_shelf_legs(self, scan):
        leg_centers = []
        threshold = 2000
        intensities = scan.intensities # Intensity threshold for reflective markers
       
        sx = 0.0
        sy = 0.0
        count = 0
        current_angle = scan.angle_min
        for i, intensity in enumerate(intensities):
            r = scan.ranges[i]
            
            # validate range: break cluster and skip if invalid
            if math.isinf(r) or math.isnan(r):
                if count > 0.0:
                    leg_centers.append((sx / count, sy / count))
                    sx = sy = count = 0.0
                current_angle += scan.angle_increment
                continue
            
            if intensity >= threshold:
                sx += r * math.cos(current_angle)
                sy += r * math.sin(current_angle)
                count += 1
            else:
                if count > 0:
                    leg_centers.append((sx/count, sy/count))
                    sx = sy = count = 0.0
            current_angle += scan.angle_increment

        # Handle any left over cluster
        if count > 0:
            leg_centers.append((sx / count, sy / count))
        
        return leg_centers

    def publish_cart_frame(self, legs, laser_frame, timestamp):

        if len(legs) != 2:
            self.get_logger().warn(f"Expected exactly 2 legs, but found {len(legs)}. Skipping TF publication.")
            return False

        cart_frame_laser = PointStamped()
        cart_frame_laser.header.frame_id = laser_frame
        cart_frame_laser.header.stamp = timestamp  #  timestamp - we use multithreading
        cart_frame_laser.point.x = (legs[0][0] + legs[1][0]) / 2
        cart_frame_laser.point.y = (legs[0][1] + legs[1][1]) / 2
        cart_frame_laser.point.z = 0.0
        
        # Leg points in laser frame
        shelf_leg1_laser = PointStamped(header=cart_frame_laser.header, point=Point(x=legs[0][0], y=legs[0][1], z=0.0))
        shelf_leg2_laser = PointStamped(header=cart_frame_laser.header, point=Point(x=legs[1][0], y=legs[1][1], z=0.0))

        try:
            transform = self.tf_buffer.lookup_transform("map", laser_frame, timestamp, timeout=Duration(seconds=0.5))

            shelf_leg1_map = do_transform_point(shelf_leg1_laser, transform)
            shelf_leg2_map = do_transform_point(shelf_leg2_laser, transform)
            cart_frame_map = do_transform_point(cart_frame_laser, transform)

            # Compute yaw along the leg line (from leg1 to leg2)
            dy = shelf_leg2_map.point.y - shelf_leg1_map.point.y
            dx = shelf_leg2_map.point.x - shelf_leg1_map.point.x
            leg_yaw = math.atan2(dy, dx)

            # Face "into" the cart: perpendicular to leg line.
            # +pi/2 for one direction (e.g., inward)
            face_yaw = leg_yaw + math.pi / 2
            
            chart_frame_tf = TransformStamped()
            chart_frame_tf.header.stamp = self.get_clock().now().to_msg()
            chart_frame_tf.header.frame_id = "map"
            chart_frame_tf.child_frame_id = "cart_frame"
            chart_frame_tf.transform.translation.x = cart_frame_map.point.x
            chart_frame_tf.transform.translation.y = cart_frame_map.point.y
            chart_frame_tf.transform.translation.z = cart_frame_map.point.z
            quat = quaternion_from_euler(0, 0, face_yaw)
            chart_frame_tf.transform.rotation.x = quat[0]
            chart_frame_tf.transform.rotation.y = quat[1]
            chart_frame_tf.transform.rotation.z = quat[2]
            chart_frame_tf.transform.rotation.w = quat[3]

            self.tf_static_broadcaster.sendTransform(chart_frame_tf)
            self.rate.sleep()   # Wait briefly for TF update
            self.get_logger().info("Successfully published cart_frame TF.")
            return True
        except TransformException as ex:
            self.get_logger().error(f"Failed to publish cart_frame TF: {ex}")
            return False

    # TODO: Rewrite this to single go with +0.5 
    # TODO: Use navigator to do the moves instead of the custom code 
    # TODO: write code to do back up and not to do back up, just set new footprint 
    #.      and move out of the position 
    def run_attach_algorithm(self):
        self.get_logger().info("Run attach algorithm!")
        # Go to the between legs and then a bit under 
        robot_frame = "robot_base_footprint"
        cart_frame = "cart_frame"

        kp_dist = 0.5
        kp_yaw = 2.0
        v_min, v_max = 0.1, 0.25  # min and max linear velocity
        w_max = 1.0  # max angular velocity
        stop_dist = 0.01  # stop distance
        target_offset = 0.4 # Advance distance
        yaw_gate = 0.5  # ~30deg: rotate-in-place when misaligned
        phase = 1 # Start Phase 1

        while rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    robot_frame, cart_frame, rclpy.time.Time(), Duration(seconds=0.1)
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF {robot_frame} to {cart_frame} failed: {ex}")
                return False

            dx = tf.transform.translation.x
            dy = tf.transform.translation.y
            error_distance = math.hypot(dx, dy)
            if phase == 1:
                # Phase 1: Approach cart_frame position with heading-based yaw
                error_yaw = math.atan2(dy, dx)
                self.get_logger().debug(
                    f"Distance to cart: {error_distance:.3f} Yaw: {error_yaw:.3f}, dx={dx:.3f} dy={dy:.3f}"
                )
                if dx >= 0.0 and abs(dx) <= stop_dist and abs(dy) <= stop_dist:
                    self.get_logger().info(f"Reached the cart frame. ({error_distance:.3f} m)")
                    phase = 2  # Transition to phase 2
                    continue  # Skip to next iteration for fresh TF in phase 2
            
            if phase == 2:
                # Phase 2: Advance additional 40cm while aligning orientation
                quat = [
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w,
                ]
                _, _, error_yaw = euler_from_quaternion(quat)
                self.get_logger().debug(f"Distance to cart_frame: {error_distance:.3f}")
                if error_distance >= target_offset:
                    self.get_logger().info(f"Advance {error_distance}m complete.")
                    break
            
            cmd = Twist()
            cmd.linear.x = max(v_min, min(kp_dist * error_distance, v_max))
            cmd.angular.z = max(-w_max, min(kp_yaw * error_yaw, w_max))

            if phase == 1 and abs(error_yaw) > yaw_gate:
                cmd.linear.x = 0.0  # turn in place first
            
            self.cmd_pub.publish(cmd)
            self.rate.sleep()
        
        stop = Twist()
        self.cmd_pub.publish(stop)
        
        # Phase 3: Rotate under the shelf 
        self.rotate_degrees()

        # Phase 3: Rise up the shelf
        self.liftup_shelf()
        return True

    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def liftup_shelf(self):
        msg = String()  # Empty message
        for _ in range(3):
            self.liftup_pub.publish(msg)
        self.get_logger().info("Lifted the shelf up.")
    
    def rotate_degrees(self, degrees=180):
        robot_frame = "robot_base_footprint"
        odom_frame = "odom"
        initial_yaw = None  
        target_yaw = None
        yaw_tol = 0.05 # Tolerance about 3 degrees
        w_max = 1.0  # max angular velocity
        kp_yaw = 2.0 

        while rclpy.ok():
            try:
                tf_odom_to_robot = self.tf_buffer.lookup_transform(
                    odom_frame, robot_frame, rclpy.time.Time(), Duration(seconds=0.1)
                )
            except TransformException as ex:
                self.get_logger().warn(f"TF {odom_frame} to {robot_frame} failed: {ex}")
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
            self.get_logger().debug(f"Rotation error_yaw: {error_yaw:.3f}")
            if abs(error_yaw) <= yaw_tol:
                self.get_logger().info(f"{degrees}-degree rotation complete.")
                break

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = max(-w_max, min(kp_yaw * error_yaw, w_max))

            self.cmd_pub.publish(cmd)
            self.rate.sleep()
        
        stop = Twist()
        self.cmd_pub.publish(stop)

class WarehouseOrchestrator(Node):
    """
    Drives to waypoints and calls pick/place services between moves. 
    """

    WAYPOINTS = {
        # x, y, yaw (rad) 
        "init_position": (0.0, 0.0, 0.0),
        # "init_position": (5.569, -1.357, -1.655),
        "loading_position":  (5.6,  0.0,  math.radians(-90)), # position
        "shipping_position": (2.5,  1.3,  math.radians(90)),
    }

    def __init__(self):
        super().__init__('warehouse_mission_orchestrator')

        # Create Basic Navigator
        self.navigator_ = BasicNavigator()
        self.set_initial_pose()
        self.navigator_.waitUntilNav2Active() # Wait for navigation to activate fully

        self.global_footprint_pub = self.create_publisher(Polygon, '/global_costmap/footprint', 1)
        self.local_footprint_pub = self.create_publisher(Polygon, '/local_costmap/footprint', 1)


        self._startup_timer  = self.create_timer(0.5, self.start_once)
        self.get_logger().info("Warehouse Orchestrator Node is Ready!")

    def set_initial_pose(self):
        x, y, yaw = self.WAYPOINTS['init_position']
        self.initial_pose_ = make_pose(x, y, yaw, 'map')
        self.initial_pose_.header.stamp = self.get_clock().now().to_msg()
        self.navigator_.setInitialPose(self.initial_pose_)
    
    def start_once(self):
        # stop the timer so it runs only once
        if hasattr(self, "_startup_timer") and self._startup_timer is not None:
            self._startup_timer.cancel()
            self.destroy_timer(self._startup_timer)
            self._startup_timer = None
        
        
        self.run_sequence()
        # if not self.goto('shipping_position'): return
        # self.goto('init_position')
        # self.get_logger().info("Mission complete ✅")
        # TODO: 1. Write first command to go to the loading position 
        #2. Write second command to go to the shipping position 
        #3. Write third command to go to the initi position 
        #4. Write PickUp Service Server to pick up the the shelf 
        #5. Write PutDown Service Server to put down the shelf
        #6. Write service clients
        #7. Test that 
    def run_sequence(self):
        # TODO: 
        self.get_logger().info("Execute the seuquence!")
        if not self.goto('loading_position'): return
        if not self.call_service('pickup_shelf'): 
            self.goto('initial_position') 
            return
        self.update_footprint(True)
        self.update_controller_critics(True)
        # self.update_obstacle_ranges(True)
        # self.navigator_.toggleCollisionMonitor(False)
        # self.move_back()
        # self.navigator_.toggleCollisionMonitor(True)
       
    
    def update_footprint(self, attached=False):
        footprint = Polygon()
        if attached:
            footprint.points = [
                Point32(x=0.45, y=0.41, z=0.0), #front_left
                Point32(x=0.45, y=-0.41, z=0.0), #front_right
                Point32(x=-0.45, y=-0.41, z=0.0), #back_right
                Point32(x=-0.45, y=0.41, z=0.0), #back_left
            ]
        else:
            n = 36
            for i in range(n):
                theta = 2 * math.pi * i / n
                footprint.points.append(Point32(x=0.25 * math.cos(theta), y=0.25*math.sin(theta), z=0.0))
        
        self.global_footprint_pub.publish(footprint)
        self.local_footprint_pub.publish(footprint)
        self.get_logger().info(f'Updated robot footprint to attached: {attached}')

    def update_obstacle_ranges(self, attached: bool):
        """
        Tweak ranges based on whether the shelf is attached.
        Adjust the numbers to your geometry.
        """
        # Compute near clearance from lidar to the closest protrusion in scan plane
        # e.g., when attached the shelf front is ~0.40 m ahead of lidar → set min ≈ 0.43
        if attached:
            min_r = 0.5  # meters (lidar→shelf + safety)
            max_r = 2.5    # meters (indoor)
        else:
            min_r = 0.08   # typical lidar minimum + margin
            max_r = 2.5

        ray_min = 0.0
        ray_max = max_r + 0.5

        params_dict = {
            'obstacle_layer.scan.obstacle_min_range':  float(min_r),
            'obstacle_layer.scan.obstacle_max_range':  float(max_r),
            'obstacle_layer.scan.raytrace_min_range':  float(ray_min),
            'obstacle_layer.scan.raytrace_max_range':  float(ray_max),
        }

        for costmap in ['local_costmap/local_costmap', 'global_costmap/global_costmap']:
            self.update_parameters(costmap, params_dict)
    
    def update_controller_critics(self, attached=False):

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

    def update_parameters(self, node_name, params_dict):
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

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            self.get_logger().info(f"Set parameters for {node_name}")
        else:
            self.get_logger().warn(f"Failed to set parameters for {node_name}")

    def move_back(self):
        # TODO: Implement move back 
        self.navigator_.backup(backup_dist=0.3, backup_speed=0.15, time_allowance=6)  # non-blocking call
        while not self.navigator_.isTaskComplete():
            fb = self.navigator_.getFeedback()  # optional: check fb.distance_traveled, etc.
            if fb: self.get_logger().info(f'Backing up: {fb.distance_traveled:.2f} m')

        result = self.navigator_.getResult()
        ok = (result == TaskResult.SUCCEEDED)
        if not ok:
            self.get_logger().error(f'BackUp failed: result={result}')
        else:
            self.get_logger().info('BackUp succeeded')
        return ok

    def goto(self, name: str) -> bool:
        x, y, yaw = self.WAYPOINTS[name]
        pose = make_pose(x, y, yaw, "map")
        pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info(f"Go to {name}: x={x:.2f}, y={y:.2f}, angle={math.degrees(yaw):.1f}")
        self.navigator_.goToPose(pose)
        i = 0
        while not self.navigator_.isTaskComplete():
            i = i + 1
            feedback = self.navigator_.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f"Estimated time of arrival at {name}" +
                f" for robot: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} seconds.")
       
        result = self.navigator_.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Arrived")
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f'Task go to {name} cancelled, returning to initial position...')
            self.initial_pose_.header.stamp = self.get_clock().now().to_msg()
            self.navigator_.goToPose(self.initial_pose_)
            return False
        else:
            self.get_logger().error(f"Task go to {name} failed!")
            return False
    
    def call_service(self, srv_name):
        client = self.create_client(Trigger, srv_name)  # Adapt to your srv type
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Service {srv_name} not available')
            return

        request = Trigger.Request()
        future = client.call_async(request)
        done = Event()
        response: Optional[Trigger.Response] = None
        exception: Optional[BaseException] = None

        def _on_done(f):
            nonlocal response, exception
            try:
                response = f.result()
            except Exception as e:
                exception = e
            finally:
                done.set()

        future.add_done_callback(_on_done)

        if not done.wait(20.0):
            return False, 'timeout waiting for response'

        if exception is not None:
            return False, f'exception from service: {exception!r}'

        return bool(response.success), response.message
    

def main():
    rclpy.init()
    # three nodes, one process
    pickup = PickUpService()
    # putdown = PutdownService()
    orchestrator = WarehouseOrchestrator()
    exec = MultiThreadedExecutor(num_threads=4)
    exec.add_node(pickup)
    # exec.add_node(putdown)
    exec.add_node(orchestrator)
    try:
        exec.spin()
    finally:
        # for n in (pickup, putdown, orchestrator):
        for n in (pickup, orchestrator):
            n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()