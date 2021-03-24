#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

import os

import launch
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    package_dir = get_package_share_directory('robots')

    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(package_dir, 'worlds', 'apartment.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments={
            'executable': 'webots_differential_drive_node',
            'world': os.path.join(package_dir, 'worlds', 'complete_apartment.wbt'),
            'node_parameters': os.path.join(package_dir, 'resource', 'tiago.yaml'),
            'use_sim_time': use_sim_time
        }.items()
    )

    head2camera = launch_ros.actions.Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      arguments=['0.0', '0.0', '0.00', '.0', '0.0', '3.1415', 'head_2_link', 'camera'],
      parameters=[{'use_sim_time': use_sim_time}],
    )
    head2rangefinder = launch_ros.actions.Node(
      package='tf2_ros',
      executable='static_transform_publisher',
    arguments=['0.0', '0.0', '0.00', '0.0', '0.0', '3.1415', 'head_2_link', 'range-finder'],
      parameters=[{'use_sim_time': use_sim_time}]
    )

    # Rviz node
    use_rviz = launch.substitutions.LaunchConfiguration('rviz', default=False)
    rviz_config = os.path.join(get_package_share_directory('robots'), 'resource', 'odometry.rviz')
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=launch.conditions.IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pointcloud_xyz = launch_ros.actions.ComposableNodeContainer(
      name='container',
      namespace='',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=[
          # Driver itself
          launch_ros.descriptions.ComposableNode(
              package='depth_image_proc',
              plugin='depth_image_proc::PointCloudXyzrgbNode',
              name='point_cloud_xyz_node',
              remappings=[('rgb/camera_info', '/camera/camera_info'),
                          ('rgb/image_rect_color', '/camera/image_raw'),
                          ('depth_registered/image_rect', '/range_finder/image_depth'),
                          ('points', '/depth_registered/points')],
              parameters=[{'use_sim_time': use_sim_time}]
          ),
      ],
      parameters=[{'use_sim_time': use_sim_time}],
      output='screen',
    )
    
    nav2_bringup_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
      launch_arguments={'namespace': namespace,
                        'use_namespace': use_namespace,
                        'slam': slam,
                        'map': map_yaml_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'default_bt_xml_filename': default_bt_xml_filename,
                        'autostart': autostart}.items())

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    
    ld.add_action(webots)
    ld.add_action(nav2_bringup_cmd)
    
    return ld
