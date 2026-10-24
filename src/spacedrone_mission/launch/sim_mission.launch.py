'''
Launches everything on the ROS side for the sim mission:
    gz bridge (camera + clock) -> aruco_detector -> tool_registry -> mission_node

Start the sim first in another terminal:  ./sim/run_sim.sh
Then:  ros2 launch spacedrone_mission sim_mission.launch.py

args:
    viewer:=true         opens rqt_image_view on the debug image
    map_file:=<path>     where the tool map gets saved (default ~/spacedrone_tool_map.yaml)
    load_map:=true       start with the tools from the last flight (for go_to_tool without searching)
    auto_start:=false    dont take off until /spacedrone/start_mission is called
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mission_share = get_package_share_directory('spacedrone_mission')
    vision_share = get_package_share_directory('spacedrone_vision')

    mission_cfg = os.path.join(mission_share, 'config', 'mission.yaml')
    bridge_cfg = os.path.join(mission_share, 'config', 'gz_bridge.yaml')
    tools_cfg = os.path.join(vision_share, 'config', 'tools.yaml')

    viewer = LaunchConfiguration('viewer')
    map_file = LaunchConfiguration('map_file')
    load_map = LaunchConfiguration('load_map')
    auto_start = LaunchConfiguration('auto_start')
    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('map_file', default_value=os.path.expanduser('~/spacedrone_tool_map.yaml')),
        DeclareLaunchArgument('load_map', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            parameters=[{'config_file': bridge_cfg}],
            output='screen',
        ),

        Node(
            package='spacedrone_vision',
            executable='aruco_detector',
            name='aruco_detector',
            parameters=[tools_cfg, use_sim_time],
            output='screen',
        ),

        Node(
            package='tool_registry',
            executable='tool_registry_node',
            name='tool_registry',
            parameters=[mission_cfg, use_sim_time,
                        {'map_file': map_file,
                         'load_existing_map': PythonExpression(["'", load_map, "' == 'true'"])}],
            output='screen',
        ),

        Node(
            package='spacedrone_mission',
            executable='mission_node',
            name='mission_node',
            parameters=[mission_cfg, use_sim_time,
                        {'auto_start': PythonExpression(["'", auto_start, "' == 'true'"])}],
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            arguments=['/spacedrone/debug_image'],
            condition=IfCondition(viewer),
        ),
    ])
