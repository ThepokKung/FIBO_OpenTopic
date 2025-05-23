#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.substitutions import TextSubstitution 
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_sim = get_package_share_directory('robot_gazebo')

    model_path =  pkg_robot_sim + '/models'

    def world_file_launch_configuration(context):
        file = os.path.join(pkg_robot_sim, 'worlds', context.launch_configurations['room'] + '.world')
        if not os.path.exists(file):
            raise FileNotFoundError(f"No such file or directory: {file}")
        return [SetLaunchConfiguration('world', file)]

    world_file_launch_configuration_arg = OpaqueFunction(function=world_file_launch_configuration)
    
    room_arg = DeclareLaunchArgument('room', 
                default_value=TextSubstitution(text='warehouse_aruco_no_actor'), 
                description='Use empty, cave or roblab to load a TUW environment')

    return LaunchDescription([
        room_arg,
        world_file_launch_configuration_arg,
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'verbose': 'true'}.items(),
        ) 
    ])