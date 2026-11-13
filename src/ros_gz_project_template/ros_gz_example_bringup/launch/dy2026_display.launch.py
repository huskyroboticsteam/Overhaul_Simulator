# Copyright 2026 Overhaul
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

"""Visualize the dy2026 master assembly (Onshape URDF export) in Rerun.

Start the viewer on your machine (`rerun`), then run this launch file.
From the Docker container the default rerun_url reaches the host's viewer.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

MODEL_NAME = 'dy2026_masterassembly'


def generate_launch_description():
    pkg_project_description = get_package_share_directory('ros_gz_example_description')

    urdf_file = os.path.join(
        pkg_project_description, 'models', MODEL_NAME, 'urdf', f'{MODEL_NAME}.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # The Onshape exporter writes mesh URIs as package://dy2026_masterassembly/...,
    # but the model is installed inside ros_gz_example_description, so point the
    # URIs at the installed location instead of editing the generated URDF.
    robot_desc = robot_desc.replace(
        f'package://{MODEL_NAME}/',
        f'package://ros_gz_example_description/models/{MODEL_NAME}/')

    use_gui = LaunchConfiguration('use_gui')

    # Slider GUI for moving the joints by hand
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    # Publishes all joints at zero when the GUI is disabled
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_desc},
        ],
    )

    # Anchor the URDF's root link to a map frame
    map_to_root = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_root',
        arguments=['--frame-id', 'map', '--child-frame-id', 'root'],
    )

    # Stream the model to a Rerun viewer (C++ node, needs the Rerun C++ SDK at build time)
    urdf_to_rerun = Node(
        package='ros_gz_example_bringup',
        executable='urdf_to_rerun',
        name='urdf_to_rerun',
        output='screen',
        parameters=[{
            'rerun_url': LaunchConfiguration('rerun_url'),
            'application_id': MODEL_NAME,
            'fixed_frame': 'map',
            'entity_root': MODEL_NAME,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rerun_url',
                              default_value='rerun+http://host.docker.internal:9876/proxy',
                              description='gRPC address of the Rerun viewer.'),
        DeclareLaunchArgument('use_gui', default_value='false',
                              description='Use joint_state_publisher_gui sliders '
                                          '(needs the container desktop).'),
        joint_state_publisher_gui,
        joint_state_publisher,
        robot_state_publisher,
        map_to_root,
        urdf_to_rerun,
    ])
