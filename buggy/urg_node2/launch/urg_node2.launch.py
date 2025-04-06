# Copyright 2022 eSOL Co.,Ltd.
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
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.actions import Node
import xacro


USE_SIM_TIME = False
PACKAGE_NAME = 'urg_node2'

package_dir = get_package_share_directory(PACKAGE_NAME)

robot_urdf_xacro = os.path.join(package_dir, 'resource', 'descriptions', 'robot.urdf.xacro')
robot_description = xacro.process_file(robot_urdf_xacro)

mapper_params_online_async_yaml = os.path.join(package_dir, 'config', 'slam_toolbox', 'mapper_params_online_async.yaml')
config_rviz = os.path.join(package_dir, 'config', 'rviz', 'config.rviz')
params_ether_yaml = os.path.join(package_dir, 'config', 'params_ether.yaml')
robot_localization_yaml = os.path.join(package_dir, 'config', 'robot_localization', 'robot_localization.yaml')


def generate_launch_description():
    with open(params_ether_yaml, 'r') as params:
        config_params = yaml.safe_load(params)['urg_node2']['ros__parameters']

    # urg_node2をライフサイクルノードとして起動
    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=LaunchConfiguration('node_name'),
        remappings=[('scan', LaunchConfiguration('scan_topic_name'))],
        parameters=[config_params],
        namespace='',
        output='screen',
    )

    lidar_odometry_node = LifecycleNode(
        executable='lidar_odometry_node', 
        package='urg_node2', 
        name='lidar_odometry_node', 
        namespace='', 
        output='screen', 
    )

    robot_state_publisher = Node(
        executable='robot_state_publisher', 
        package='robot_state_publisher', 
        parameters=[{
            'use_sim_time': False, 
            'robot_description': robot_description.toxml(), 
        }], 
        output='screen', 
    )

    robot_controller_node = Node(
        executable='robot_controller_node.py', 
        package=PACKAGE_NAME, 
        name='robot_controller_node', 
        parameters=[{'use_sim_time': USE_SIM_TIME}], 
        output='screen', 
    )

    static_transforms = [
        ['map', 'odom'], 
    ]
    
    static_transform_nodes = []

    for transform in static_transforms:
        static_transform_nodes.append(Node(
            executable='static_transform_publisher', 
            package='tf2_ros', 
            name='static_transform_publisher', 
            parameters=[{'use_sim_time': USE_SIM_TIME}], 
            arguments=['0', '0', '0', '0', '0', '0'] + transform, 
            output='screen', 
        ))

    robot_localization_node = Node(
        executable='ekf_node', 
        package='robot_localization', 
        name='ekf_filter_node', 
        parameters=[
            robot_localization_yaml, 
            {'use_sim_time': USE_SIM_TIME}, 
        ], 
        output='screen', 
    )

    async_slam_toolbox_node = LifecycleNode(
        executable='async_slam_toolbox_node', 
        package='slam_toolbox', 
        name='slam_toolbox', 
        namespace='', 
        parameters=[
          mapper_params_online_async_yaml, 
          {
            'use_sim_time': USE_SIM_TIME, 
            'use_lifecycle_manager': False, 
          }
        ], 
        output='screen', 
    )

    localization_slam_toolbox_node = LifecycleNode(
        executable='localization_slam_toolbox_node', 
        package='slam_toolbox', 
        name='slam_toolbox', 
        namespace='', 
        parameters=[
          mapper_params_online_async_yaml, 
          {
            'use_sim_time': USE_SIM_TIME, 
            'use_lifecycle_manager': False, 
          }
        ], 
        output='screen', 
    )

    rviz = Node(
        executable='rviz2', 
        package='rviz2', 
        name='rviz2', 
        namespace='', 
        arguments=['-d', config_rviz], 
        output='screen', 
    )

    joint_state_publisher_gui = Node(
        executable='joint_state_publisher_gui', 
        package='joint_state_publisher_gui', 
        name='joint_state_publisher_gui', 
        namespace='', 
        output='screen', 
    )

    # Unconfigure状態からInactive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # Inactive状態からActive状態への遷移（auto_startがtrueのとき実施）
    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    # パラメータについて
    # auto_start      : 起動時自動でActive状態まで遷移 (default)true
    # node_name       : ノード名 (default)"urg_node2"
    # scan_topic_name : トピック名 (default)"scan" *マルチエコー非対応*

    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'), 
        DeclareLaunchArgument('node_name', default_value='urg_node2'), 
        DeclareLaunchArgument('scan_topic_name', default_value='scan'), 
        lifecycle_node, 
        lidar_odometry_node, 
        robot_state_publisher, 
        robot_controller_node, 
        robot_localization_node, 
        async_slam_toolbox_node, 
        # localization_slam_toolbox_node, 
        rviz, 
        joint_state_publisher_gui, 
        urg_node2_node_configure_event_handler, 
        urg_node2_node_activate_event_handler, 
    ])  # + static_transform_nodes
