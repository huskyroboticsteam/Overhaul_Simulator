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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction

from launch_ros.actions import Node, PushRosNamespace

MODEL_NAME = 'dy2026_simple'


def generate_launch_description():
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(
        pkg_project_description, 'models', MODEL_NAME, 'urdf', f'{MODEL_NAME}.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    use_gui = LaunchConfiguration('use_gui')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )

    print(urdf_file)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'demo',
            '-file', urdf_file,
            '-name', 'swerve_drive',
            '-z', '0.2'
        ],
        output='screen'
    )

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

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Rover nodes. The following nodes normally run on the rover hower interface
    # with out emulated robot interfaces

    rover = Node(
        package='rover',
        executable='Rover',
        name='rover',
        output='both',
        parameters=[{'peripheral': 'launch'}],)

    drive = Node(
        package='rover',
        executable='Drive',
        name='Drive',
        output='both',)

    return LaunchDescription([
        DeclareLaunchArgument('rerun_url',
                              default_value='rerun+http://host.docker.internal:9876/proxy',
                              description='gRPC address of the Rerun viewer.'),
        DeclareLaunchArgument('use_gui', default_value='false',
                              description='Use joint_state_publisher_gui sliders '
                                          '(needs the container desktop).'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        GroupAction([
            PushRosNamespace(MODEL_NAME),
            joint_state_publisher_gui,
            joint_state_publisher,
            robot_state_publisher,
            map_to_footprint,
            rviz,
            gz_sim,
            rover,
            drive,
            bridge,
        ]),
        # Incredibly dumb but for some reason URDF cannot pull in the 
        TimerAction(
            period=5.0,
            actions=[spawn]
        ),
    ])
