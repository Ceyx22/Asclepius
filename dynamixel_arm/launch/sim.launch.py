
import os
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    pkg_name = 'dynamixel_arm'
    file_subpath = 'urdf/acps_bot.urdf.xacro'
    
    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir(pkg_name), 'rviz/viewrobot.rviz')

    # Locate xacro file
    xacro_file = os.path.join(pkgdir(pkg_name),file_subpath)
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # gazebo param 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkgdir('ros_ign_gazebo'), 'launch'), '/ign_gazebo.launch.py']),
        )

    # joint_node = Node(
    #         package="joint_state_publisher_gui",
    #         executable="joint_state_publisher_gui",
    #         name="joint_state_publisher_gui",
    #         output="screen")
    
    robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config}],
            output="screen")

    create = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=["-topic", "robot_description",
                   "-entity", "acps_bot"],
        output="screen"
    )

    joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "--spin-time", "120",
             "joint_state_broadcaster"],
        output="screen"
    )
    joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_trajectory_controller"],
        output="screen"
    )
    velocity_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "configure",
             "velocity_controller"],
        output="screen"
    )

    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rvizcfg],
            output="screen")
    
    create_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=create,
                on_exit=[joint_state_broadcaster],
            )
        )
    
    joint_state_broadcaster_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller],
            )
        )

    joint_trajectory_handler = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_trajectory_controller,
                on_exit=[velocity_controller],
            )
        )
    
    return LaunchDescription([
        
        create_handler,
        joint_state_broadcaster_handler,
        joint_trajectory_handler, 
        rviz_node,
        gazebo, 
        create,
        robot_state_publisher_node,
    ])
