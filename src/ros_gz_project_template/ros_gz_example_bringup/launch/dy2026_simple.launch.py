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

"""Visualize the simplified dy2026 rover (boxes and cylinders) in Rerun.

The URDF is generated from the Onshape export by
ros_gz_example_description/tools/simplify_dy2026.py.

Start the viewer on your machine (`rerun`), then run this launch file.
From the Docker container the default rerun_url reaches the host's viewer.

Everything runs in the /dy2026_simple namespace (robot_description, joint_states), so
it can run alongside dy2026_display.launch.py without the two models mixing; TF is
global, and the link names don't overlap.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace

MODEL_NAME = 'dy2026_simple'


def generate_launch_description():
    pkg_project_description = get_package_share_directory('ros_gz_example_description')

    urdf_file = os.path.join(
        pkg_project_description, 'models', MODEL_NAME, 'urdf', f'{MODEL_NAME}.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

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

    # The rover stands on the map origin
    map_to_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_footprint',
        arguments=['--frame-id', 'map', '--child-frame-id', 'base_footprint'],
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
        GroupAction([
            PushRosNamespace(MODEL_NAME),
            joint_state_publisher_gui,
            joint_state_publisher,
            robot_state_publisher,
            map_to_footprint,
            urdf_to_rerun,
        ]),
    ])
