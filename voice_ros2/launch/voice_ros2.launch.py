#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get path to the default parameter file in the voice_ros2 package.
    voice_ros2_share = get_package_share_directory('voice_ros2')
    default_param_file = os.path.join(voice_ros2_share, 'params', 'default.yaml')

    # Launch argument for parameter file (default: voice_ros2/params/default.yaml)
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Full path to the ROS2 parameters file to use'
    )

    # Node for Coqui TTS
    tts_node = Node(
        package='coqui_tts_ros2',
        executable='tts_node',
        name='tts',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/tts_state', '/tts_state')
        ]
    )

    tts_voice_clone_node = Node(
        package='coqui_tts_ros2',
        executable='tts_node',
        name='tts_voice_clone',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('~/tts_state', '/tts_state')
        ]
    )

    # Node for Vosk Speech Recognition
    vosk_node = Node(
        package='vosk_ros2',
        executable='vosk_ros2_node',
        name='vosk',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            # Remap tts status to disable listening when tts is talking
            ('~/tts_status_in', '/tts_state')
        ]
    )

    vosk_gpsr_node = Node(
        package='vosk_ros2',
        executable='vosk_ros2_node',
        name='vosk_gpsr',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            # Remap tts status to disable listening when tts is talking
            ('~/tts_status_in', '/tts_state')
        ]
    )

    return LaunchDescription([
        params_file_arg,
        tts_node,
        tts_voice_clone_node,
        vosk_node,
        vosk_gpsr_node
    ])

if __name__ == '__main__':
    generate_launch_description()
