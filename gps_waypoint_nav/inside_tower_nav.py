import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math
import numpy as np

class DroneSafetyController(Node):
    def __init__(self):
        super().__init__('inside_tower_nav_node')

        # Declare all parameters with default values
        self.declare_parameters(namespace='',
            parameters=[
                ('cmd_vel_topic', '/cmd_vel'),
                ('cmd_vel_safe_topic', '/cmd_vel_safe'),
                ('laserscan_topic', '/scan'),
                ('obstacle_threshold', 0.6),  # meters
                ('negligible_difference', 0.1),  # meters
                ('forward_speed', 0.3),  # m/s
                ('lateral_speed', 0.2),  # m/s
                ('rotation_speed', 0.3),  # rad/s
                ('search_rotation_speed', 0.5)  # rad/s when searching
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
        self.SEARCH_ROTATION_SPEED = self.get_parameter('search_rotation_speed').value

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

        # Variables
        self.latest_cmd_vel = Twist()
        self.laserscan_data = None

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def laserscan_callback(self, msg):
        self.laserscan_data = msg
        ranges = np.array(msg.ranges)

        # Define regions (indices)
        regions = {
            'front_right': (0, 55),
            'right': (55, 125),
            'back_right': (125, 180),
            'back_left': (180, 235),
            'left': (235, 305),
            'front_left': (305, 360)
        }

        # Get valid ranges for each region
        region_data = {}
        for name, (start, end) in regions.items():
            region_ranges = ranges[start:end]
            valid_ranges = region_ranges[(region_ranges > 0) & (region_ranges < float('inf'))]
            region_data[name] = {
                'min_distance': np.min(valid_ranges) if valid_ranges.size > 0 else float('inf'),
                'average_distance': np.mean(valid_ranges) if valid_ranges.size > 0 else float('inf')
            }

        # Find R1 and R2 (two regions with smallest min distances)
        sorted_regions = sorted(region_data.items(), key=lambda x: x[1]['min_distance'])
        R1_name, R1_data = sorted_regions[0]
        R2_name, R2_data = sorted_regions[1]

        self.get_logger().info(
            f"Closest regions: {R1_name} ({R1_data['min_distance']:.2f}m), "
            f"{R2_name} ({R2_data['min_distance']:.2f}m)"
        )

        # Initialize command with all zeros
        # cmd_vel = Twist()
        # cmd_vel.linear.x = 0.0
        # cmd_vel.linear.y = 0.0
        # cmd_vel.angular.z = 0.0

        # # Case 1: Both R1 and R2 are front regions
        # if {R1_name, R2_name} == {'front_left', 'front_right'}:
        #     if abs(R1_data['min_distance'] - R2_data['min_distance']) > self.NEGLIGIBLE_DIFF:
        #         # Move laterally toward the region with larger distance
        #         if R1_data['min_distance'] > R2_data['min_distance']:
        #             cmd_vel.linear.y = -self.LATERAL_SPEED  # Move left (toward front_left)
        #         else:
        #             cmd_vel.linear.y = self.LATERAL_SPEED   # Move right (toward front_right)
        #     else:
        #         # Safe to move forward
        #         cmd_vel.linear.x = self.FORWARD_SPEED
        # else:
        #     # Case 2: Not both front regions - rotate toward R1
        #     if R1_name.endswith('right'):
        #         cmd_vel.angular.z = -self.ROTATION_SPEED  # Rotate right (clockwise)
        #     elif R1_name.endswith('left'):
        #         cmd_vel.angular.z = self.ROTATION_SPEED   # Rotate left (counter-clockwise)
        #     else:
        #         # For back regions, still rotate toward R1
        #         if 'right' in R1_name:
        #             cmd_vel.angular.z = -self.ROTATION_SPEED
        #         else:
        #             cmd_vel.angular.z = self.ROTATION_SPEED

        # # Publish the command
        # self.cmd_vel_pub.publish(cmd_vel)

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