import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


package_dir = get_package_share_directory('urg_node2')

robot_urdf_xacro = os.path.join(package_dir, 'resource', 'descriptions', 'robot.urdf.xacro')
robot_description = xacro.process_file(robot_urdf_xacro)

robot_rviz = os.path.join(package_dir, 'config', 'rviz', 'robot.rviz')


def generate_launch_description():
    robot_state_publisher = Node(
        executable='robot_state_publisher', 
        package='robot_state_publisher', 
        parameters=[{
            'use_sim_time': False, 
            'robot_description': robot_description.toxml(), 
        }], 
        output='screen', 
    )

    rviz = Node(
        executable='rviz2', 
        package='rviz2', 
        name='rviz2', 
        namespace='', 
        arguments=['-d', robot_rviz], 
        output='screen', 
    )

    joint_state_publisher_gui = Node(
        executable='joint_state_publisher_gui', 
        package='joint_state_publisher_gui', 
        name='joint_state_publisher_gui', 
        namespace='', 
        output='screen', 
    )

    return LaunchDescription([
        robot_state_publisher, 
        rviz, 
        joint_state_publisher_gui, 
    ])
