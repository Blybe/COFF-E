"""Nav2 + RViz for the simulated Avular Origin One.

Starts, next to the Gazebo simulation (separate container, same ROS graph):
  - pointcloud_to_laserscan: /robot/lidar/points -> /scan
  - Nav2 bringup (map_server, AMCL, planner, controller, behaviours, BT)
  - relay /cmd_vel -> /robot/cmd_vel (Nav2 output -> simulated robot)
  - RViz with the Nav2 panel (shown in the browser via noVNC)

  - Cerebra waypoints, paths and zones as RViz markers (/waypoints)
  - Nav2 keep-out filter for Cerebra no_go_area zones

Configuration comes from environment variables, mostly written by
tools/cerebra_import.py (selected.env, sourced by the container command):
  NAV_MAP          Nav2 map yaml
  KEEPOUT_MASK     Nav2 keep-out mask yaml (only if the map has no-go zones)
  CEREBRA_SCENE    scene.json with waypoints, paths and zones
  ROBOT_POSE_X/Y/YAW  Spawn pose, used as AMCL initial pose
  NAV_PARAMS       Override file merged onto the Nav2 defaults
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

NAV_DIR = os.path.dirname(os.path.abspath(__file__))


class NoAliasDumper(yaml.SafeDumper):
    """ROS 2 params files must not contain YAML anchors/aliases (&id001)."""

    def ignore_aliases(self, data):
        return True


def deep_merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def add_keepout_filter(params, mask_yaml):
    keepout = {
        'plugin': 'nav2_costmap_2d::KeepoutFilter',
        'enabled': True,
        'filter_info_topic': '/costmap_filter_info',
    }
    for costmap in ('global_costmap', 'local_costmap'):
        p = params[costmap][costmap]['ros__parameters']
        p['filters'] = ['keepout_filter']
        p['keepout_filter'] = keepout
    params['filter_mask_server'] = {'ros__parameters': {
        'use_sim_time': True, 'yaml_filename': mask_yaml,
        'topic_name': '/keepout_filter_mask', 'frame_id': 'map'}}
    params['costmap_filter_info_server'] = {'ros__parameters': {
        'use_sim_time': True, 'type': 0, 'filter_info_topic': '/costmap_filter_info',
        'mask_topic': '/keepout_filter_mask', 'base': 0.0, 'multiplier': 1.0}}


def build_params(keepout_mask):
    bringup_dir = get_package_share_directory('nav2_bringup')
    with open(os.path.join(bringup_dir, 'params', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)
    override_file = os.environ.get(
        'NAV_PARAMS', os.path.join(NAV_DIR, 'origin_nav_params.yaml'))
    with open(override_file) as f:
        deep_merge(params, yaml.safe_load(f) or {})

    params['amcl']['ros__parameters']['initial_pose'] = {
        'x': float(os.environ.get('ROBOT_POSE_X', '0.0')),
        'y': float(os.environ.get('ROBOT_POSE_Y', '0.0')),
        'z': 0.0,
        'yaw': float(os.environ.get('ROBOT_POSE_YAW', '0.0')),
    }
    params.setdefault('pointcloud_to_laserscan', {}).setdefault(
        'ros__parameters', {})['use_sim_time'] = True
    if keepout_mask:
        add_keepout_filter(params, keepout_mask)

    out = tempfile.NamedTemporaryFile(
        'w', suffix='_nav2_params.yaml', delete=False)
    yaml.dump(params, out, Dumper=NoAliasDumper)
    out.close()
    return out.name


def generate_launch_description():
    map_yaml = os.environ.get('NAV_MAP', '').strip()
    keepout_mask = os.environ.get('KEEPOUT_MASK', '').strip()
    if keepout_mask and not os.path.isfile(keepout_mask):
        keepout_mask = ''
    scene_file = os.environ.get('CEREBRA_SCENE', '/tmp/cerebra/scene.json')
    params_file = build_params(keepout_mask)
    bringup_dir = get_package_share_directory('nav2_bringup')

    actions = [
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[params_file],
            remappings=[('cloud_in', '/robot/lidar/points'),
                        ('scan', '/scan')],
            output='screen'),
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=['/cmd_vel', '/robot/cmd_vel'],
            parameters=[{'use_sim_time': True}],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(NAV_DIR, 'origin_nav.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ]

    if map_yaml and os.path.isfile(map_yaml):
        actions.insert(0, LogInfo(msg=f'[origin_nav] Using map {map_yaml}'))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': 'True',
                'autostart': 'True',
            }.items()))
        if keepout_mask:
            actions.insert(0, LogInfo(msg=f'[origin_nav] Keep-out zones from {keepout_mask}'))
            actions += [
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='filter_mask_server',
                    parameters=[params_file],
                    output='screen'),
                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='costmap_filter_info_server',
                    parameters=[params_file],
                    output='screen'),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_costmap_filters',
                    parameters=[{'use_sim_time': True, 'autostart': True,
                                 'node_names': ['filter_mask_server',
                                                'costmap_filter_info_server']}],
                    output='screen'),
            ]
    else:
        actions.insert(0, LogInfo(msg=(
            f'[origin_nav] No Nav2 map found ("{map_yaml}"). Nav2 is NOT '
            'started; only RViz runs. Set CEREBRA_MAP in .env (see README).')))

    if os.path.isfile(scene_file):
        actions.append(ExecuteProcess(
            cmd=['python3', os.path.join(NAV_DIR, 'cerebra_markers.py'), scene_file],
            output='screen'))

    return LaunchDescription(actions)
