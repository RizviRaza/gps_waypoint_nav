from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    key_path = PathJoinSubstitution([
        FindPackageShare("gps_waypoint_nav"),
        "key",
        "ros-team-key.json",
    ])

    return LaunchDescription([
        # set environment variable for Google credentials
        SetEnvironmentVariable(
            name="GOOGLE_APPLICATION_CREDENTIALS",
            value=key_path
        ),

        DeclareLaunchArgument("mode", default_value="gps"),
        DeclareLaunchArgument("project_id", default_value="cviss-waypoint-control"),
        DeclareLaunchArgument("gps_subscription", default_value="gps-sub"),
        DeclareLaunchArgument("utm_subscription", default_value="utm-sub"),
        DeclareLaunchArgument("colmap_subscription", default_value="colmap-sub"),
        DeclareLaunchArgument("pull_limit", default_value="5"),
        DeclareLaunchArgument("publish_ros", default_value="true"),

        Node(
            package="gps_waypoint_nav",
            executable="pubsub_wp_ros",
            name="pubsub_wp_ros",
            output="screen",
            parameters=[{
                "mode": LaunchConfiguration("mode"),
                "project_id": LaunchConfiguration("project_id"),
                "gps_subscription": LaunchConfiguration("gps_subscription"),
                "utm_subscription": LaunchConfiguration("utm_subscription"),
                "colmap_subscription": LaunchConfiguration("colmap_subscription"),
                "pull_limit": LaunchConfiguration("pull_limit"),
                "publish_ros": LaunchConfiguration("publish_ros"),
            }],
        ),
    ])
