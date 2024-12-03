import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    jdamr200_urdf = os.path.join(
        get_package_share_directory('jdamr200_description'),
        'urdf',
        'jdamr200.urdf')
    with open(jdamr200_urdf, 'r') as file:
        jdamr200_desc = file.read()

    ld14lidar_launch_file_dir = os.path.join(
        get_package_share_directory('ldlidar_sl_ros2'),
        'launch'
    )
    
    robot_param = {'robot_description': jdamr200_desc}
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
         # Include the launch file that starts the ydlidar launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ld14lidar_launch_file_dir, '/ld14.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_param, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='jdamr200_node',
            executable='jdamr200_node',
            output='screen',
        ),
    ])