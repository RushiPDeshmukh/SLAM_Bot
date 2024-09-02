# Author: Addison Sears-Collins
# Date: August 31, 2021
# Description: Launch a basic mobile robot
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
  pkg_share = FindPackageShare(package='slam_bot_description').find('slam_bot_description') 
  model = os.path.join(pkg_share, 'models/slam_bot.urdf')
  rviz_config_file = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
  robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', model])}]
    )
  
  start_car_controller = Node(
    package='car_control',
    executable='car_controller',
    name='car_controller'
  )

  start_camera_imu_publisher = Node(
    package='camera_pkg',
    executable='imu_img_depth_pub',
    name='camera_imu_publisher'
  )

  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])    

  start_visual_odometry = Node(
    package='odometry',
    executable='visual_odom',
    name='visual_odometry'
  )

  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path])

  # Create the launch description and populate
  ld = LaunchDescription()

  # Add any actions
  ld.add_action(start_robot_state_publisher_cmd)
  # ld.add_action(start_car_controller)
  # ld.add_action(start_camera_imu_publisher)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_rviz_cmd)
  # ld.add_action(start_visual_odometry)
  return ld
