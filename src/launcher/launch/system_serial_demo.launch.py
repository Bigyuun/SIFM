#!/usr/bin/env python3
# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  
  return LaunchDescription([
    
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     [get_package_share_directory('fts_pkg'), '/launch/_launch.py']),
    # ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [get_package_share_directory('serial_pkg'), '/launch/_launch_demo.py']),
    ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [get_package_share_directory('kinematics_control_pkg'), '/launch/_launch.py'])
    ),
    
    IncludeLaunchDescription(      
      PythonLaunchDescriptionSource(
        [get_package_share_directory('tcp_pkg'), '/launch/_launch.py'])
    ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [get_package_share_directory('gui_py_pkg'), '/launch/_launch.py']),
    ),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        [get_package_share_directory('record_pkg'), '/launch/_launch.py']),
    ),
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource(
    #     [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
    #     launch_arguments={
    #       'rgb_camera.profile': '640,480,30',
    #       'depth_module.profile': '640,480,30'
    #     }.items()
    # ),
    
  ])