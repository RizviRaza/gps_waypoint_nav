import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.ndimage import median_filter
# from pynput import keyboard 
import keyboard

class DroneSafetyController(Node):
    def __init__(self):
        super().__init__('drone_safety_controller')

        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('cmd_vel_topic', '/cmd_vel'),
                ('cmd_vel_safe_topic', '/mavic_1/cmd_vel'),
                ('laserscan_topic', '/scan'),
                ('obstacle_threshold', 0.6),  # meters
                ('negligible_difference', 0.1),  # meters
                ('forward_speed', 0.3),  # m/s
                ('lateral_speed', 0.2),  # m/s
                ('rotation_speed', 0.2),  # rad/s
                ('median_window_size', 7),  # Filter window size (odd)
                ('min_obstacle_points', 2)  # Min consecutive points to consider as obstacle
            ])

        # Get parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_safe_topic = self.get_parameter('cmd_vel_safe_topic').value
        self.laserscan_topic = self.get_parameter('laserscan_topic').value
        self.OBSTACLE_THRESHOLD = self.get_parameter('obstacle_threshold').value
        self.NEGLIGIBLE_DIFF = self.get_parameter('negligible_difference').value
        self.FORWARD_SPEED = self.get_parameter('forward_speed').value
        self.LATERAL_SPEED = self.get_parameter('lateral_speed').value
        self.ROTATION_SPEED = self.get_parameter('rotation_speed').value
        self.MEDIAN_WINDOW = self.get_parameter('median_window_size').value
        self.MIN_POINTS = self.get_parameter('min_obstacle_points').value

        # Subscribers and Publisher
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.laserscan_sub = self.create_subscription(
            LaserScan, self.laserscan_topic, self.laserscan_callback, qos_profile=qos_profile)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_safe_topic, 10)

        self.latest_cmd_vel = Twist()

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def filter_ranges(self, ranges):
        """Apply median filtering and handle invalid values."""
        # Replace inf/nan with large value for filtering
        clean_ranges = np.where(np.isfinite(ranges), ranges, 10e6)
        # Apply median filter
        filtered = median_filter(clean_ranges, size=self.MEDIAN_WINDOW, mode='mirror')
        # Restore inf for originally invalid points
        return np.where(np.isfinite(ranges), filtered, float('inf'))

    def has_valid_obstacle(self, region_ranges):
        """Check if at least MIN_POINTS consecutive points are below threshold."""
        count = 0
        for r in region_ranges:
            if r < self.OBSTACLE_THRESHOLD and not math.isinf(r):
                count += 1
                if count >= self.MIN_POINTS:
                    return True
            else:
                count = 0
        return False

    def center_between_obstacles(self, R1, R2):
        """Generate velocity commands to center drone between two front obstacles"""
        # Parameters (tunable)
        TARGET_ANGLE = 30.0      # ±30° from center = 60° total spread
        MIN_DISTANCE = 1.0       # meters
        ANGLE_TOLERANCE = 5.0    # degrees
        DIST_TOLERANCE = 0.1     # meters
        
        cmd = Twist()
        conditions_met = True
        
        # Convert angles to standard frame (-180° to 180°)
        r1_angle = R1['angle']
        r2_angle = R2['angle']
        
        # 1. Check midpoint alignment
        mid_angle = (r1_angle + r2_angle) / 2
        if abs(mid_angle) > ANGLE_TOLERANCE/2:
            cmd.angular.z = -self.ROTATION_SPEED if mid_angle > 0 else self.ROTATION_SPEED
            conditions_met = False
            self.get_logger().info(f"Correcting heading: turning {'right' if mid_angle > 0 else 'left'}")
        
        # 2. Check angular spread
        current_spread = abs(r1_angle - r2_angle)
        if current_spread < 2*TARGET_ANGLE - ANGLE_TOLERANCE:
            cmd.linear.x = self.FORWARD_SPEED * 0.5  # Forward to widen angles
            conditions_met = False
            self.get_logger().info("Moving forward to widen angles")
        elif current_spread > 2*TARGET_ANGLE + ANGLE_TOLERANCE:
            cmd.linear.x = -self.FORWARD_SPEED * 0.5  # Backward to narrow angles
            conditions_met = False
            self.get_logger().info("Moving backward to narrow angles")
        
        # 3. Check distance balance
        distance_diff = R1['min_distance'] - R2['min_distance']
        if abs(distance_diff) > DIST_TOLERANCE:
            cmd.linear.y = self.LATERAL_SPEED if distance_diff > 0 else -self.LATERAL_SPEED
            conditions_met = False
            self.get_logger().info(f"Strafing {'left' if distance_diff > 0 else 'right'} to balance distances")
        
        # 4. Check minimum distance
        avg_distance = (R1['min_distance'] + R2['min_distance']) / 2
        if avg_distance < MIN_DISTANCE - DIST_TOLERANCE:
            cmd.linear.x = -abs(cmd.linear.x) if cmd.linear.x != 0 else -self.FORWARD_SPEED * 0.5
            conditions_met = False
            self.get_logger().info("Too close - moving backward")
        elif avg_distance > MIN_DISTANCE + DIST_TOLERANCE:
            cmd.linear.x = abs(cmd.linear.x) if cmd.linear.x != 0 else self.FORWARD_SPEED * 0.5
            conditions_met = False
            self.get_logger().info("Too far - moving forward")
        
        # If all conditions are met, publish zero velocity
        if conditions_met:
            self.get_logger().info("Optimal position achieved - maintaining hover")
            return Twist()  # Zero velocity
        
        return cmd

    def laserscan_callback(self, msg):
        # Convert to numpy array and apply median filtering
        raw_ranges = np.array(msg.ranges)
        ranges = self.filter_ranges(raw_ranges)

        # Define regions (indices and angle ranges)
        regions = {
            'front_right': {'indices': range(0, 55), 'angle_range': (0, 54)},
            'right': {'indices': range(55, 125), 'angle_range': (55, 124)},
            'back_right': {'indices': range(125, 180), 'angle_range': (125, 179)},
            'back_left': {'indices': range(180, 235), 'angle_range': (180, 234)},
            'left': {'indices': range(235, 305), 'angle_range': (235, 304)},
            'front_left': {'indices': range(305, 360), 'angle_range': (305, 359)}
        }

        # Analyze regions
        valid_regions = []
        for name, region_info in regions.items():
            region_ranges = ranges[list(region_info['indices'])]
            valid_points = [(i, r) for i, r in enumerate(region_ranges) 
                        if not (math.isinf(r) or math.isnan(r))]
            
            if len(valid_points) > 0:
                # Find the point with minimum distance in this region
                min_idx, min_dist = min(valid_points, key=lambda x: x[1])
                
                # Check for nearby similar points (cluster detection)
                cluster_points = 1
                cluster_threshold = min_dist * 0.2  # 20% of min distance
                
                # Check points before min index
                i = min_idx - 1
                while i >= 0 and abs(region_ranges[i] - min_dist) <= cluster_threshold:
                    cluster_points += 1
                    i -= 1
                
                # Check points after min index
                i = min_idx + 1
                while i < len(region_ranges) and abs(region_ranges[i] - min_dist) <= cluster_threshold:
                    cluster_points += 1
                    i += 1
                
                # Only consider if we have a cluster (not isolated point)
                if cluster_points >= self.MIN_POINTS:
                    # Calculate angle and convert to counter-clockwise if > 180
                    raw_angle = (region_info['angle_range'][0] + min_idx) * msg.angle_increment
                    angle = math.degrees(raw_angle)
                    if angle > 180:
                        # angle = 360 - angle  # Convert to counter-clockwise equivalent
                        angle = angle - 360
                        angle_direction = "CCW"
                    else:
                        angle_direction = "CW"
                    
                    # angle = math.degrees((region_info['angle_range'][0] + min_idx) * msg.angle_increment)
                    valid_regions.append({
                        'name': name,
                        'min_distance': min_dist,
                        'angle': angle,
                        'cluster_size': cluster_points
                    })

        # Sort regions by minimum distance
        valid_regions.sort(key=lambda x: x['min_distance'])

        # Print information for R1 and R2
        if len(valid_regions) >= 2:
            R1, R2 = valid_regions[0], valid_regions[1]
            if "right" in R1['name'] and "left" in R2['name']:
                # Swap them if they're in the wrong order
                R1, R2 = R2, R1
                
            if {"front_right", "front_left"} == {R1['name'], R2['name']}:
                # self.get_logger().info(f"R1 has '{R1['name']}', R2 has '{R2['name']}'")
                self.get_logger().info(
                f"R1: {R1['name']} | Distance: {R1['min_distance']:.2f}m | "
                f"Angle: {R1['angle']:.1f}° | Cluster: {R1['cluster_size']} points\n"
                f"R2: {R2['name']} | Distance: {R2['min_distance']:.2f}m | "
                f"Angle: {R2['angle']:.1f}° | Cluster: {R2['cluster_size']} points"
                )

                self.latest_cmd_vel = self.center_between_obstacles(R1, R2)
                # self.cmd_vel_pub.publish(cmd)

                # self.latest_cmd_vel.angular.z = 0.0  # Rotate right
                self.cmd_vel_pub.publish(self.latest_cmd_vel)
                self.get_logger().info("Stop Rotating\n")
            else:
                self.get_logger().info(f"Unexpected names: R1='{R1['name']}', R2='{R2['name']}'. Rotating\n")
                
                if any('left' in r['name'].lower() for r in [R1, R2]):
                    self.latest_cmd_vel.angular.z = self.ROTATION_SPEED  # Positive for left
                elif any('right' in r['name'].lower() for r in [R1, R2]):
                    self.latest_cmd_vel.angular.z = -self.ROTATION_SPEED  # Negative for right

                self.cmd_vel_pub.publish(self.latest_cmd_vel)
               

        elif len(valid_regions) == 1:
            R1 = valid_regions[0]
            self.get_logger().info(
                f"Single obstacle detected:\n"
                f"R1: {R1['name']} | Distance: {R1['min_distance']:.2f}m | "
                f"Angle: {R1['angle']:.1f}° | Cluster: {R1['cluster_size']} points"
            )
        else:
            self.get_logger().warn("No valid obstacle clusters detected")


def main(args=None):
    rclpy.init(args=args)
    node = DroneSafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()