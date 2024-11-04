import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import time
import threading
import math

class DroneWaypointNavigator(Node):
    def __init__(self):
        super().__init__('drone_waypoint_navigator')
        
        # Subscribers
        self.gps_subscription = self.create_subscription(
            Float64MultiArray,
            '/mavic_1/GPS2',
            self.gps_callback,
            10)
        
        self.waypoint_subscription = self.create_subscription(
            NavSatFix,
            '/waypoint',
            self.waypoint_callback,
            10)
        
        # Publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, '/mavic_1/cmd_vel', 10)
        
        # Store current GPS and waypoint data
        self.current_gps = None
        self.target_altitude = None
        self.target_longitude = None
        self.target_latitude = None
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Start GPS listener thread
        self.gps_thread = threading.Thread(target=self.gps_listener_thread)
        self.gps_thread.daemon = True
        self.gps_thread.start()

        # Navigation thread reference
        self.navigation_thread = None

    def gps_callback(self, msg):
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
        # When a waypoint is received, set the target altitude, longitude, and latitude
        with self.lock:
            self.target_altitude = msg.altitude
            self.target_longitude = msg.longitude
            self.target_latitude = msg.latitude
        # self.get_logger().info(f'Waypoint received with target altitude: {self.target_altitude}, longitude: {self.target_longitude}, latitude: {self.target_latitude}')
        
        # Start navigation to the waypoint in a separate thread
        if self.navigation_thread is None or not self.navigation_thread.is_alive():
            self.navigation_thread = threading.Thread(target=self.navigate_to_waypoint)
            self.navigation_thread.start()

    def gps_listener_thread(self):
        # Continuously spin to receive GPS updates
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def navigate_to_waypoint(self):
        if self.current_gps is None or self.target_altitude is None:
            self.get_logger().warning('GPS or waypoint data is not available yet.')
            return
        
        self.get_logger().info('Navigating to waypoint...')
        velocity_command = Twist()
        
        # Generate velocity commands to reach the target altitude
        while True:
            with self.lock:
                if self.current_gps is None:
                    continue
                alt_diff = self.target_altitude - self.current_gps['altitude']
                if abs(alt_diff) <= 0.1:
                    break
                velocity_command.linear.z = max(min(0.5 * alt_diff, 0.5), -0.5)  # Proportional control
            self.cmd_vel_publisher.publish(velocity_command)
            time.sleep(0.1)
        
        self.get_logger().info('Reached waypoint, maintaining position for 5 seconds...')
        end_time = time.time() + 5  # Maintain position for 5 seconds with corrections
        while time.time() < end_time:
            with self.lock:
                alt_diff = self.target_altitude - self.current_gps['altitude']
                velocity_command.linear.z = max(min(0.5 * alt_diff, 0.5), -0.5)
            self.cmd_vel_publisher.publish(velocity_command)
            time.sleep(0.1)
        
        # Stop publishing velocities to allow the drone to hover
        velocity_command.linear.z = 0.0
        self.cmd_vel_publisher.publish(velocity_command)
        self.get_logger().info('Hovering at waypoint.')

        # Start turning towards target longitude and latitude
        self.turn_to_target()

    def turn_to_target(self):
        if self.current_gps is None or self.target_longitude is None or self.target_latitude is None:
            self.get_logger().warning('GPS or target data is not available yet for turning.')
            return
        
        self.get_logger().info('Turning towards target longitude and latitude...')
        velocity_command = Twist()
        
        while True:
            with self.lock:
                if self.current_gps is None:
                    continue

                self.get_logger().info(f'Current GPS: {self.current_gps}')
                # Calculate the desired heading to the target
                delta_long = self.target_longitude - self.current_gps['longitude']
                delta_lat = self.target_latitude - self.current_gps['latitude']
                desired_heading = math.degrees(math.atan2(delta_long, delta_lat))

                self.get_logger().info(f'desired_heading: {desired_heading}')

                if desired_heading < 0:
                    desired_heading += 360
                current_heading = self.current_gps['heading']
                heading_diff = desired_heading - current_heading
                
                # Normalize the heading difference to the range [-180, 180]
                # if heading_diff > 180:
                #     heading_diff -= 360
                # elif heading_diff < -180:
                #     heading_diff += 360
                
                # If the heading difference is small enough, stop turning
                if abs(heading_diff) <= 5.0:
                    break
                
                # Apply proportional control to turn towards the desired heading
                velocity_command.angular.z = abs(max(min(0.01 * heading_diff, 0.5), -0.5))
            self.cmd_vel_publisher.publish(velocity_command)
            time.sleep(0.1)
        
        # Stop publishing velocities to allow the drone to stabilize
        velocity_command.angular.z = 0.0
        self.cmd_vel_publisher.publish(velocity_command)
        self.get_logger().info('Reached desired heading, stabilizing.')


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
