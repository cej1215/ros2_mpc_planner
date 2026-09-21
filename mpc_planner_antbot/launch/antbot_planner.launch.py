#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mpc_planner_antbot')

    # bridge_params_default = os.path.join(pkg_share, 'config', 'bridge_params.yaml')
    
    # nav2 params 및 코스트맵 전용 yaml 파일 경로
    nav2_params_default = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    local_costmap_params_default = os.path.join(pkg_share, 'config', 'local_costmap.yaml')

    map_default = '/home/cyj/antbot_ws/src/antbot/antbot_navigation/maps/new_depot.yaml'

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=map_default,
                              description='map yaml Absolute Path'),

        DeclareLaunchArgument('nav2_params', default_value=nav2_params_default),
      
        DeclareLaunchArgument('local_costmap_params', default_value=local_costmap_params_default),

        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),

        DeclareLaunchArgument('costmap_topic', default_value='/local_costmap/costmap'),
        DeclareLaunchArgument('costmap_node_name', default_value='local_costmap'),

        DeclareLaunchArgument('planner_id', default_value='GridBased'),
        DeclareLaunchArgument('target_frame', default_value='odom'),

        DeclareLaunchArgument('use_localization', default_value='true',
                              description='false 면 map_server/amcl 을 띄우지 않음'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')
    local_costmap_params = LaunchConfiguration('local_costmap_params')

    # map server, amcl configure → activate
    localization = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_localization')),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[nav2_params, {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': LaunchConfiguration('map'),
                }],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            ),
        ],
    )

    # global planner
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    planner_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    # local planner
    mpc_costmap = Node(
        package='mpc_planner_antbot',   
        executable='costmap_node',     
        name=LaunchConfiguration('costmap_node_name'),
        output='screen',
        parameters=[
            local_costmap_params,       
            {'use_sim_time': use_sim_time},
        ],
    )

    # lifecycle manager for local costmap
    costmap_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mpc_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [('local_costmap/local_costmap')],
            'bond_timeout': 0.0
        }],
    )

    path_bridge = Node(
        package='mpc_planner_antbot',
        executable='nav2_path_bridge',
        name='nav2_path_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planner_id': LaunchConfiguration('planner_id'),
            'planner_action_name': '/compute_path_to_pose',
            'target_frame': LaunchConfiguration('target_frame'),
        }],
        remappings=[
            ('~/input/goal', LaunchConfiguration('goal_topic')),
            ('~/input/path', '/plan'),
            ('~/output/reference_path', '/plan_odom'),
            ('~/output/goal', '/goal_odom'),
        ],
    )

    # MPC 노드
    mpc = Node(
        package='mpc_planner_antbot',
        executable='antbot_planner',
        name='antbot_planner',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_hardcoded_path': False,
        }],
        remappings=[
            ('~/input/state',          LaunchConfiguration('odom_topic')),
            ('~/input/goal',           '/goal_odom'),
            ('~/input/reference_path', '/plan_odom'),
            ('~/input/costmap',        LaunchConfiguration('costmap_topic')),
            # ('~/input/goal_reached',   '/nav2_path_smoother/goal_reached'),
            ('~/output/command',       LaunchConfiguration('cmd_vel_topic')),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'ros2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    return LaunchDescription(args + [
        localization,
        planner,
        planner_lifecycle,
        mpc_costmap,
        costmap_lifecycle,
        # smoother,
        path_bridge,
        mpc,
        rviz,
    ])
