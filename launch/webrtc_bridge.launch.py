"""Launch file for WebRTC Bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    signaling_port_arg = DeclareLaunchArgument(
        'signaling_port',
        default_value='8080',
        description='WebSocket signaling server port'
    )

    video_topic_arg = DeclareLaunchArgument(
        'video_topic',
        default_value='/camera/image_raw',
        description='ROS2 image topic to stream'
    )

    string_topic_out_arg = DeclareLaunchArgument(
        'string_topic_out',
        default_value='/webrtc/string_out',
        description='ROS2 string topic to send to WebRTC clients'
    )

    string_topic_in_arg = DeclareLaunchArgument(
        'string_topic_in',
        default_value='/webrtc/string_in',
        description='ROS2 string topic for messages from WebRTC clients'
    )

    video_width_arg = DeclareLaunchArgument(
        'video_width',
        default_value='640',
        description='Video width'
    )

    video_height_arg = DeclareLaunchArgument(
        'video_height',
        default_value='480',
        description='Video height'
    )

    video_fps_arg = DeclareLaunchArgument(
        'video_fps',
        default_value='30',
        description='Video frames per second'
    )

    video_bitrate_arg = DeclareLaunchArgument(
        'video_bitrate',
        default_value='1000',
        description='Video bitrate in kbps'
    )

    test_publisher_arg = DeclareLaunchArgument(
        'test_publisher',
        default_value='false',
        description='Launch test publisher for demo'
    )

    web_server_arg = DeclareLaunchArgument(
        'web_server',
        default_value='false',
        description='Launch web client server'
    )

    web_server_port_arg = DeclareLaunchArgument(
        'web_server_port',
        default_value='8000',
        description='Web client server port'
    )

    # WebRTC Bridge Node
    webrtc_bridge_node = Node(
        package='webrtc_bridge',
        executable='webrtc_bridge_node',
        name='webrtc_bridge_node',
        output='screen',
        parameters=[{
            'signaling_port': LaunchConfiguration('signaling_port'),
            'video_topic': LaunchConfiguration('video_topic'),
            'string_topic_out': LaunchConfiguration('string_topic_out'),
            'string_topic_in': LaunchConfiguration('string_topic_in'),
            'video_width': LaunchConfiguration('video_width'),
            'video_height': LaunchConfiguration('video_height'),
            'video_fps': LaunchConfiguration('video_fps'),
            'video_bitrate': LaunchConfiguration('video_bitrate'),
        }]
    )

    # Get the client directory from the installed package share
    package_share_dir = get_package_share_directory('webrtc_bridge')
    client_dir = os.path.join(package_share_dir, 'client')

    # Test Publisher (optional)
    test_publisher = ExecuteProcess(
        cmd=['python3', os.path.join(client_dir, 'test_publisher.py')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('test_publisher'))
    )

    # Web Server (optional)
    web_server = ExecuteProcess(
        cmd=['python3', os.path.join(client_dir, 'serve.py'),
             '-p', LaunchConfiguration('web_server_port')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('web_server'))
    )

    return LaunchDescription([
        signaling_port_arg,
        video_topic_arg,
        string_topic_out_arg,
        string_topic_in_arg,
        video_width_arg,
        video_height_arg,
        video_fps_arg,
        video_bitrate_arg,
        test_publisher_arg,
        web_server_arg,
        web_server_port_arg,
        webrtc_bridge_node,
        test_publisher,
        web_server,
    ])
