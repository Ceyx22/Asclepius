
import os
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_name = 'dynamixel_arm'
    file_subpath = 'urdf/acps.urdf'
    
    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir(pkg_name), 'rviz/viewrobot.rviz')

    # Locate the URDF file.
    urdf = os.path.join(pkgdir(pkg_name), file_subpath)

    # Load the robot's URDF file (XML).
    with open(urdf, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description}],
            output="screen"),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rvizcfg],
            output="screen"),

        Node(
            name       = 'transform_node', 
            package    = 'dynamixel_arm',
            executable = 'transform_node',
            output     = 'screen') # 
    ])
