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

    robot_param = {'robot_description': jdamr200_desc}

    return LaunchDescription([
        # Gazebo 시작
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
        ),

        # 로봇 상태 퍼블리셔 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_param, {'use_sim_time': use_sim_time}],
        ),

        # Gazebo에 로봇 스폰
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'jdamr200'],
            output='screen'
        ),
    ])