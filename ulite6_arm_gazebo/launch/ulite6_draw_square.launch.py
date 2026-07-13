#!/usr/bin/env python3
"""Levanta la simulacion del ulite6 sobre la mesa (table.world) y ejecuta
el nodo que traza un cuadrado de 15x15 cm y un triangulo equilatero de
15 cm de altura sobre el lienzo, regresando a la posicion inicial entre
figuras. Al terminar, la simulacion queda abierta (cerrar con Ctrl+C).

Uso:
    ros2 launch ulite6_arm_gazebo ulite6_draw_square.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    add_gripper = LaunchConfiguration('add_gripper', default=True)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ulite6_arm_gazebo'), 'launch', 'ulite6_table_gazebo.launch.py'])),
        launch_arguments={
            'add_gripper': add_gripper,
        }.items(),
    )

    # el nodo espera por si mismo a robot_description, joint_states y al
    # action server del controlador antes de planificar
    draw_square_node = Node(
        package='ulite6_arm_gazebo',
        executable='draw_test.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_launch,
        draw_square_node,
    ])
