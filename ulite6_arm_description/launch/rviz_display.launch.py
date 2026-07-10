#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def evaluate_xacro(context, *args, **kwargs):
    # Obtener la ruta del xacro evaluada en tiempo de ejecución
    xacro_file_path = LaunchConfiguration('model').perform(context)
    
    # Procesar el archivo Xacro pasando el mapeo del robot
    robot_description_config = xacro.process_file(xacro_file_path, mappings={'robot_type': 'lite6'})
    robot_description_param = {'robot_description': robot_description_config.toxml()}

    # Lanzar los nodos encargados del estado del robot pasándole el parámetro ya procesado
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    return [robot_state_publisher_node, joint_state_publisher_gui_node]

def generate_launch_description():
    pkg_share = FindPackageShare('ulite6_arm_description')
    
    default_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'mech', 'ulite6_arm.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Ruta absoluta al archivo xacro del robot'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=default_rviz_config_path,
        description='Ruta absoluta al archivo de configuración de RViz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        OpaqueFunction(function=evaluate_xacro), # Importado correctamente desde launch.actions
        rviz_node
    ])