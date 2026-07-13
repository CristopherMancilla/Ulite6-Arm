#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    load_controller = LaunchConfiguration('load_controller', default=True)

    ros2_control_plugin = 'gz_ros2_control/GazeboSimSystem'

    # ros2_control params (always generated explicitly, since the ulite6
    # xacro's built-in default path points at a package that doesn't exist
    # in this workspace)
    # add_gripper is always False here: passing True makes uf_ros_lib look up
    # the nonexistent xarm_controller package; the gripper controller is
    # already defined in ulite6_controllers.yaml instead
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('ulite6_arm_gazebo'), 'config', 'ulite6_controllers.yaml'),
        prefix=prefix.perform(context),
        add_gripper=False,
        update_rate=1000,
        use_sim_time=True,
    )

    robot_description = {
        'robot_description': get_xacro_content(
            context,
            xacro_file=Path(get_package_share_directory('ulite6_arm_description')) / 'urdf' / 'ulite6_device.urdf.xacro',
            prefix=prefix,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
        )
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    gazebo_world = PathJoinSubstitution([FindPackageShare('ulite6_arm_gazebo'), 'worlds', 'table.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': [' -r -v 3 ', gazebo_world, ' --physics-engine gz-physics-bullet-featherstone-plugin'],
        }.items(),
    )

    gazebo_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'UF_ROBOT',
            '-x', '-0.2',
            '-y', '-0.5',
            '-z', '1.021',
            '-Y', '1.571',
        ],
        parameters=[{'use_sim_time': True}],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    controllers = [
        'joint_state_broadcaster',
        '{}ulite6_traj_controller'.format(prefix.perform(context)),
    ]
    if add_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}ulite6_gripper_traj_controller'.format(prefix.perform(context)))

    controller_nodes = []
    if load_controller.perform(context) in ('True', 'true'):
        for controller in controllers:
            controller_nodes.append(Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[controller, '--controller-manager', '/controller_manager'],
                parameters=[{'use_sim_time': True}],
            ))

    return [
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=robot_state_publisher_node, on_start=gazebo_launch)
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=robot_state_publisher_node, on_start=gazebo_spawn_entity_node)
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(target_action=robot_state_publisher_node, on_start=gz_bridge)
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=gazebo_spawn_entity_node, on_exit=controller_nodes)
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
