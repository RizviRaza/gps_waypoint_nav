from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='image_republish',
        #     namespace='mavic_1/decoded',
        #     output='screen',
        #     arguments=[
        #         'ffmpeg', 'compressed',
        #         '--ros-args',
        #         '--remap', 'in/ffmpeg:=/mavic_1/image/ffmpeg',
        #         '--remap', 'out:=/mavic_1/decoded'
        #     ],
        #     parameters=[
        #         {'ffmpeg_image_transport.decode.threads': 4},
        #         {'ffmpeg_image_transport.map.hevc_nvenc': 'hevc_nvenc'}
        #     ]
        # ),

        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',            # matches your node name
            namespace='',
            output='screen',
            arguments=[
                'ffmpeg', 'raw',                 # or 'compressed'
                '--ros-args',
                '--remap', 'in/ffmpeg:=/mavic_1/image/ffmpeg',
                '--remap', 'out:=/mavic_1/decoded'
            ],
            parameters=[
                {'ffmpeg_image_transport.decode.threads': 4},
                {'in.ffmpeg.map.h264': 'h264'},  # or h264_cuvid / h264_qsv / h264_v4l2m2m
                {'in.ffmpeg.decode.threads': 4}, # if declared in your build
            ]
        ),


        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt_image_view',
            output='screen',
            # arguments=['/mavic_1/decoded']
        )
    ])
