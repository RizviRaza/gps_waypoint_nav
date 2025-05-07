from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments for topic names
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/mavic_1/cmd_vel_unsafe',
        description='Topic name for incoming cmd_vel messages'
    )
    laserscan_topic_arg = DeclareLaunchArgument(
        'laserscan_topic',
        default_value='/mavic_1/horizontal_obstacle_distance',
        description='Topic name for incoming laserscan messages'
    )
    cmd_vel_safe_topic_arg = DeclareLaunchArgument(
        'cmd_vel_safe_topic',
        default_value='/mavic_1/cmd_vel',
        description='Topic name for outgoing safe cmd_vel messages'
    )

    # Node configuration
    inside_tower_nav_node = Node(
        package='gps_waypoint_nav',
        executable='inside_tower_nav',
        name='inside_tower_nav',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'laserscan_topic': LaunchConfiguration('laserscan_topic'),
            'cmd_vel_safe_topic': LaunchConfiguration('cmd_vel_safe_topic'),
        }]
    )

    # Add actions to the launch description
    return LaunchDescription([
        cmd_vel_topic_arg,
        laserscan_topic_arg,
        cmd_vel_safe_topic_arg,
        inside_tower_nav_node,
    ])