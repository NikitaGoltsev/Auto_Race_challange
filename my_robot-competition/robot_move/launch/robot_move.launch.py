import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_directory = get_package_share_directory('robot_bringup')
    camera_directory = get_package_share_directory('autorace_camera')
    move_directory = get_package_share_directory('robot_move')

    autorace_sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(robot_directory, 'launch', 'autorace_2023.launch.py')
    ))
    camera_calibration = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(camera_directory, 'launch', 'extrinsic_camera_calibration.launch.py')
    ))

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
    	output='screen',
        parameters=[{'model_path':move_directory}]
        )
    
    return LaunchDescription([
        # launches
        autorace_sim,
        camera_calibration,

        # my_nodes
        pid_reg,
        detect_line,
        detector_sign,

        TimerAction(
            period= 5.0,
            actions=[Node(
                package= 'referee_console',
                executable= 'mission_autorace_2023_referee',
                name='mission_autorace_2023_referee'
            )])
    ])
