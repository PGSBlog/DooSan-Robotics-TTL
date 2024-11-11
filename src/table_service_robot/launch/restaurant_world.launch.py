#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로
    pkg_dir = get_package_share_directory('table_service_robot')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Nav2 파라미터 파일
    nav2_params_path = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')
    
    # World 파일 경로
    world_file = os.path.join(pkg_dir, 'worlds', 'restaurant_world.world')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Turtlebot3 모델
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    
    # Turtlebot3 launch 파일 포함
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn Turtlebot3
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'turtlebot3_' + TURTLEBOT3_MODEL,
            '-file', os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
            '-x', '0.0',
            '-y', '-3.0',
            '-z', '0.01'
        ],
        output='screen'
    )
    
    # Nav2 실행
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path
        }.items()
    )
    
    # RViz2 실행
    rviz_config_dir = os.path.join(pkg_turtlebot3_gazebo, 'rviz', 'nav2_default_view.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Launch Description 생성
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(turtlebot3_gazebo)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(nav2_bringup)
    ld.add_action(rviz2)
    
    return ld
