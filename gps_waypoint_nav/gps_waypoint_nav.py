import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import threading
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class DroneWaypointNavigator(Node):
    LINEAR_SCALE_FACTOR = 15000
    ALT_VELOCITY_BOUND = 1.0    # in m/s
    VELOCITY_BOUND = 0.5        # in m/s
    INITIAL_ALTITUDE = 60.0     # in m
    HEADING_THRESHOLD = 2.0     # in degree
    ALTITUDE_THRESHOLD = 0.1    # in m
    WAYPOINT_THRESHOLD = 0.000007

    def __init__(self):
        super().__init__('drone_waypoint_navigator')
        
        # Subscribers
        self.gps_subscription = self.create_subscription(
            Float64MultiArray,
            '/mavic_1/GPS2',
            self.gps_callback,
            10)
        
        self.waypoint_subscription = self.create_subscription(
            Float64MultiArray,
            '/towereye_wp',
            self.waypoint_callback,
            10)
        
        self.origin_subscription = self.create_subscription(
            Float64MultiArray,
            '/setOriginLocation',
            self.origin_callback,
            10)
        
        self.return_home_subscription = self.create_subscription(
            Empty,
            '/mavic_1/return_home',
            self.return_home_callback,
	    qos_profile=QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
            )
        
        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/mavic_1/cmd_vel', 10)

        self.land_publisher = self.create_publisher(Empty, '/mavic_1/land', 10)
        
        # Store current GPS and waypoint data
        self.current_gps = None
        self.origin_gps = None
        self.target_altitude = None
        self.target_longitude = None
        self.target_latitude = None
        self.target_heading = None
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Start GPS listener thread
        self.gps_thread = threading.Thread(target=self.gps_listener_thread)
        self.gps_thread.daemon = True
        self.gps_thread.start()

        # Navigation thread reference
        self.navigation_thread = None

    def gps_callback(self, msg):
        # Ensure the GPS data has the correct number of elements
        if len(msg.data) < 4:
            self.get_logger().error('GPS data does not have enough elements. Expected 4.')
            return
    
        # Update the current GPS data: Lon, Lat, Alt, heading
        with self.lock:
            self.current_gps = {
                'longitude': msg.data[0],
                'latitude': msg.data[1],
                'altitude': msg.data[2],
                'heading': msg.data[3]
            }
        #self.get_logger().info(f'Current GPS: {self.current_gps}')

    def waypoint_callback(self, msg):
        if len(msg.data) < 4:
            self.get_logger().error('Waypoint data does not have enough elements. Expected 4.')
            return
        
        # When a waypoint is received, set the target longitude and latitude
        with self.lock:
            self.target_longitude = msg.data[0]
            self.target_latitude = msg.data[1]
            self.target_altitude = msg.data[2]
            self.target_heading = msg.data[3]

        # Start navigation to the waypoint in a separate thread
        if self.navigation_thread is None or not self.navigation_thread.is_alive():
            target_gps = {
                'longitude': self.target_longitude,
                'latitude': self.target_latitude,
                'altitude': self.target_altitude,
                'heading': self.target_heading
            }
            self.navigation_thread = threading.Thread(target=self.navigate_to_waypoint, args=(target_gps,))
            self.navigation_thread.start()

    def origin_callback(self, msg):
        # Ensure the GPS data has the correct number of elements
        if len(msg.data) < 4:
            self.get_logger().error('Origin data does not have enough elements. Expected 4.')
            return
    
        # Update the origin GPS data: Lon, Lat, Alt, heading
        with self.lock:
            self.origin_gps = {
                'longitude': msg.data[1],
                'latitude': msg.data[0],
                'altitude': 0.0,
                'heading': msg.data[3]
            }
        self.get_logger().info(f'Origin GPS: {self.origin_gps}')

    def return_home_callback(self, msg):
        self.get_logger().info('Return home command received.')
        if self.origin_gps is None:
            self.get_logger().warning('Origin GPS data is not available.')
            return

        # Start navigation to the origin in a separate thread
        if self.navigation_thread is None or not self.navigation_thread.is_alive():
            self.navigation_thread = threading.Thread(target=self.navigate_to_waypoint, args=(self.origin_gps,))
            self.navigation_thread.start()

    def gps_listener_thread(self):
        # Continuously spin to receive GPS updates
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def reach_target_altitude(self, target_altitude):
        self.get_logger().info(f'Reaching target altitude: {target_altitude}m...')
        velocity_command = Twist()
        
        while True:
            with self.lock:
                if self.current_gps is None:
                    continue
                alt_diff = target_altitude - self.current_gps['altitude']
                if abs(alt_diff) <= self.ALTITUDE_THRESHOLD:
                    break
                velocity_command.linear.z = max(min(0.5 * alt_diff, self.ALT_VELOCITY_BOUND), -self.ALT_VELOCITY_BOUND)
            self.cmd_vel_publisher.publish(velocity_command)
            time.sleep(0.1)
        
        # Stop altitude movement
        velocity_command.linear.z = 0.0
        self.cmd_vel_publisher.publish(velocity_command)
        self.get_logger().info('Reached target altitude.')

    def navigate_to_waypoint(self, target_gps):
        if self.current_gps is None or target_gps is None:
            self.get_logger().warning('GPS or waypoint data is not available yet.')
            return
        
        self.get_logger().info('Navigating to waypoint...')
        
        # Step 1: Reach the target altitude
        self.reach_target_altitude(self.INITIAL_ALTITUDE)
        
        # Step 2: Calculate the desired heading to the target
        with self.lock:
            delta_long = target_gps['longitude'] - self.current_gps['longitude']
            delta_lat = target_gps['latitude'] - self.current_gps['latitude']
            desired_heading = math.degrees(math.atan2(delta_long, delta_lat))
            if desired_heading < 0:
                desired_heading += 360
        
        # Step 3: Turn towards the waypoint target
        self.turn_to_target(desired_heading)
        
        # Step 4: Move towards the waypoint longitude and latitude
        self.move_to_target(target_gps['longitude'], target_gps['latitude'])

        # Step 5: Rotate to target heading
        self.turn_to_target(target_gps['heading'])
        
        # Step 6: Descend to the target altitude

        if target_gps['altitude'] == 0.0:           # Not tested yet
            self.land_publisher.publish(Empty())
            return

        self.reach_target_altitude(target_gps['altitude'])

    def turn_to_target(self, desired_heading):
        if self.current_gps is None:
            self.get_logger().warning('GPS or target data is not available yet for turning.')
            return
        
        self.get_logger().info('Turning towards target heading...')
        velocity_command = Twist()
        
        while True:
            with self.lock:
                if self.current_gps is None:
                    continue
                
                current_heading = self.current_gps['heading']
                heading_diff = (desired_heading - current_heading + 360) % 360

                # heading_diff = desired_heading - current_heading
                
                # Determine the shortest rotation direction
                if heading_diff > 180:
                    heading_diff -= 360  # Rotate counterclockwise
                
                if abs(heading_diff) <= self.HEADING_THRESHOLD:
                    break
                
                velocity_command.angular.z = -max(min(0.01 * heading_diff, self.VELOCITY_BOUND), -self.VELOCITY_BOUND)
            self.cmd_vel_publisher.publish(velocity_command)
            time.sleep(0.1)
        
        velocity_command.angular.z = 0.0
        self.cmd_vel_publisher.publish(velocity_command)
        self.get_logger().info('Reached desired heading.')

    def move_to_target(self, target_longitude, target_latitude):
        self.get_logger().info('Moving towards target longitude and latitude...')
        velocity_command = Twist()
        
        while True:
            with self.lock:
                if self.current_gps is None:
                    continue
                
                # Calculate distance to the target
                delta_long = target_longitude - self.current_gps['longitude']
                delta_lat = target_latitude - self.current_gps['latitude']
                distance = math.sqrt(delta_long**2 + delta_lat**2)
                
                if distance <= self.WAYPOINT_THRESHOLD:
                    break
                
                velocity_command.linear.x = max(min(self.LINEAR_SCALE_FACTOR * distance, self.VELOCITY_BOUND), -self.VELOCITY_BOUND)
                
                desired_heading = math.degrees(math.atan2(delta_long, delta_lat))
                if desired_heading < 0:
                    desired_heading += 360
                current_heading = self.current_gps['heading']
                heading_diff = desired_heading - current_heading

                if heading_diff > 180:
                    heading_diff -= 360  # Rotate counterclockwise
                
                if abs(heading_diff) > self.HEADING_THRESHOLD:
                    velocity_command.linear.x = 0.0
                    velocity_command.angular.z = -max(min(0.01 * heading_diff, self.VELOCITY_BOUND), -self.VELOCITY_BOUND)
                else:
                    velocity_command.angular.z = 0.0
            
            self.cmd_vel_publisher.publish(velocity_command)
            time.sleep(0.1)
        
        velocity_command.linear.x = 0.0
        velocity_command.angular.z = 0.0
        self.cmd_vel_publisher.publish(velocity_command)
        self.get_logger().info('Reached target location.')

def main(args=None):
    rclpy.init(args=args)
    
    navigator = DroneWaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down drone waypoint navigator.')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
