# Overview
This is a ROS2 package for GPS Waypoint navigation with velocity commands.
The package was initially developed for a DJI Mavic 3E drone, but it can be used with any drone as long as the ROS Topics are respected.

# Build Environment
- Ubuntu 20.04
- ROS2 Humble

# Features
- Receives the drone's current GPS location with heading.
- Recevies the drone's home GPS coordinate.
- Receives the goal GPS coodinate with heading.
- Publishes velocity commands for navigating the drone to reach the goal GPS coordinate.
    - The drone takesoff and reaches a height of 60m.
    - Then, the drone orients itself toward the waypoint Long. and Lat.
    - Then, generates linear velocity to reach the Long. Lat., while also correcting the heading on its way.
    - After reaching the Long. Lat., the drone orients towards the goal heading.
    - At last, the drone decends to the goal Altitude.
    - After reaching the Goal waypoint, another waypoint can be published or return home can be triggered.
- Can trigger return home.

# Topics Subscribed
- `/mavic_1/GPS2`: GPS Topic containining Longitude, Latitude, Altitude, and Heading (+ve degrees from North)
    - Type: `Float64MultiArray`
    - Data: 
      'longitude': msg.data[0],
      'latitude': msg.data[1],
      'altitude': msg.data[2],
      'heading': msg.data[3]

- `towereye_wp`: Target GPS Waypoint
    - Type: `Float64MultiArray`
    - Data:
      'longitude': msg.data[0],
      'latitude': msg.data[1],
      'altitude': msg.data[2],
      'heading': msg.data[3]

- `setOriginLocation`: Drone Home GPS coordinate
    - Type: `Float64MultiArray`
    - Data:
      'longitude': msg.data[0],
      'latitude': msg.data[1],
      'altitude': msg.data[2],
      'heading': msg.data[3]

- `/mavic_1/return_home`: Topic to trigger retur home
    - Type: Empty

# Topics Published

- `/mavic_1/cmd_vel`: Velocity commands for navigation
    - Type: `geometry_msgs/Twist`

- `/mavic_1/land`: Trigger Autoland
    - Type: `Empty`

# Files in the package
- `gps_waypoint_nav.py`: The main Python ROS2 node responsible for navigation.
- `gps_wp_controller.py`: The launch file for the package.
    - Launches the `gps_waypoint_nav.py` node.
    - Launches the `image_transport` ROS2 package's `republish` node to subscribe to the ROS2 image topic of encoding `ffmpeg`, and decode and publish a `raw` image on a new ROS2 topic.
    - Launches the `rqt_image_view` ROS2 package's `rqt_image_view` node to visualize the raw image topic.
    - Launches 3 `topic_tools` ROS2 package's `relay` nodes to republish the ROS2 topics coming from the VPN to the host.

# How to run
-  Clone the repo to your ROS2 workspace.
- Use `rosdep` to satisfy the dependencies.
- Build the package.
- Source the builds.
- Launch the `gps_wp_conteroller.py` launch file.

# Notes
- Use the `DroneWaypointNavigator` class variables in the `gps_wp_controller.py` node to cofigure the behavior.
- The `relay` nodes in the launch are includes due to the use of a VPN for remote operations. Correctly configure the VPN.
- Example DDS configuration file is included in the `dds_config` folder.