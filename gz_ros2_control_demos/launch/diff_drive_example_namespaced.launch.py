# Copyright 2024 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('gz_ros2_control_demos')

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    def robot_state_publisher(context):
        description_format = LaunchConfiguration('description_format').perform(context)
        # Get URDF or SDF via xacro
        xacro_processed = xacro.process(
            os.path.join(
                pkg_share,
                description_format,
                f'test_diff_drive.xacro.{description_format}'
            ),
            mappings={'namespace': 'r1'}
        )
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='r1',
            output='screen',
            parameters=[{'robot_description': xacro_processed}]
        )
        return [node_robot_state_publisher]

    robot_controllers = os.path.join(
        pkg_share,
        'config',
        'diff_drive_controller.yaml'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/r1/controller_manager'
        ],
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            robot_controllers,
            '-c', '/r1/controller_manager'
        ],
    )

    # Launch just the Gazebo server as a composable node.
    gz_server = GzServer(
        world_sdf_file='empty.sdf',
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )

    gz_gui = ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen')

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='r1',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true'],
    )

    # Setup ros_gz_bridge to bridge topics between ROS and Gazebo.
    # It is launched as a composable node in the container created by the Gazebo server.
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=os.path.join(pkg_share, 'config', 'ros_gz_bridge_config.yaml'),
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    ld = LaunchDescription([
        gz_server,
        gz_gui,
        gz_spawn_entity,
        ros_gz_bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='sdf',
            description='Robot description format to use, urdf or sdf'),
    ])
    ld.add_action(OpaqueFunction(function=robot_state_publisher))
    return ld
