import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pid_reg = Node(
    	package= 'robot_move',
    	executable= 'pid_reg',
    	namespace='drive',
    	name= 'pid',
    	output='screen'
    )
    
    detect_line = Node(
    	package= 'robot_move',
    	executable= 'detect_line',
    	namespace='detect',
        name='line',
    	output='screen'
    )
    
    detector_sign = Node(
    	package= 'robot_move',
    	executable= 'detector_sign',
    	namespace='detect',
        name='line',
    	output='screen'
    )
    
    return LaunchDescription([
        pid_reg,
        detect_line,
        detector_sign,
    ])
