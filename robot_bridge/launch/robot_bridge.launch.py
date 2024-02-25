#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro   
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler 
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
    
def generate_launch_description():

    #=====================================================================#
    #======================== Generate RVIZ ==============================#
    #=====================================================================#
    pkg = get_package_share_directory('example_description')
    rviz_path = os.path.join(pkg,'config','_display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    path_description = os.path.join(pkg,'robot','visual','robot.xacro')
    robot_desc_xml = xacro.process_file(path_description).toxml()

    parameters = [{'robot_description':robot_desc_xml}]
    #parameters.append({'frame_prefix':namespace+'/'})
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    #=====================================================================#
    #==================== Generate Robot Bridge ==========================#
    #=====================================================================#

    pkg = get_package_share_directory('robot_bridge')
    rviz_path = os.path.join(pkg,'config','_display.rviz')

    DiffDriveRobot = Node(
                    package='robot_bridge',
                    executable='diff_drive_robot.py')
    
    CommandOdom = Node(
                    package='robot_bridge',
                    executable='cmd_odom.py')
    
    #=====================================================================#
    #====================== Generate IMU Node ============================#
    #=====================================================================#

    # pkg = get_package_share_directory('calibration_gen')
    # rviz_path = os.path.join(pkg,'config','_display.rviz')

    imuread_node = Node(
                    package='calibration_gen',
                    executable='imuread_node.py')
    
    #=====================================================================#
    #================= Generate Robot localization =======================#
    #=====================================================================#

    # pkg = get_package_share_directory('calibration_gen')
    # rviz_path = os.path.join(pkg,'config','_display.rviz')
    print(os.path.join(get_package_share_directory('robot_localization'), 'launch'), '/ekf.launch.py')

    ekf_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('robot_localization'), 'launch'), '/ekf.launch.py']),
             )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        rviz, 
        robot_state_publisher,
        joint_state_publisher,
        DiffDriveRobot,
        CommandOdom,
        # imuread_node,
        ekf_node,
    ])