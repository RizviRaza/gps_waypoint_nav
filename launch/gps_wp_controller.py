from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_waypoint_nav',
            executable='gps_waypoint_nav',
            name='gps_waypoint_nav',
            output='screen'
        ),

        Node(
            package='image_transport',
            executable='republish',
            name='image_republish',
            output='screen',
            arguments=[
                'ffmpeg',
                'in/ffmpeg:=/mavic_1/image/ffmpeg',
                'raw',
                'out:=mavic_1/decoded'
            ],
            parameters=[
                {'ffmpeg_image_transport.map.hevc_nvenc': 'hevc'}
            ]
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            arguments=['/mavic_1/decoded']
        ),

        Node(
            package='topic_tools',             # The package name
            executable='relay',                # The executable name
            name='gimbal_vel_cmd_relay',       # Optional: node name
            output='screen',                   # Output configuration
            arguments=[
                '/remote/gimbal_vel_cmd',      # Source topic
                '/mavic_1/gimbal_vel_cmd'      # Target topic
            ]
        ),

        Node(
            package='topic_tools',             # The package name
            executable='relay',                # The executable name
            name='gimbal_cmd_relay',       # Optional: node name
            output='screen',                   # Output configuration
            arguments=[
                '/remote/gimbal_cmd',      # Source topic
                '/mavic_1/gimbal_cmd'      # Target topic
            ]
        ),

        Node(
            package='topic_tools',             # The package name
            executable='relay',                # The executable name
            name='mavic_cmd_relay',       # Optional: node name
            output='screen',                   # Output configuration
            arguments=[
                '/remote/cmd_vel',      # Source topic
                '/mavic_1/cmd_vel'      # Target topic
            ]
        ),
    ])
