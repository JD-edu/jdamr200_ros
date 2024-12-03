import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    jdamr200_urdf = os.path.join(
        get_package_share_directory('jdamr200_description'),
        'urdf',
        'jdamr200.urdf')
    with open(jdamr200_urdf, 'r') as file:
        jdamr200_desc = file.read()

    rviz_config_file = os.path.join(get_package_share_directory('jdamr200_bringup'), 'rviz','jdamr200_display_rviz.rviz')
   
    robot_param = {'robot_description': jdamr200_desc}
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_param, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])