"""Start Gazebo with the hospital sink world and spawn the Clearpath robot.

clearpath_gz's own simulation.launch.py only accepts its built-in world names,
so this includes its two halves directly:
  * gz_sim.launch.py      gets the full path to our .sdf (minus the extension)
  * robot_spawn.launch.py gets the world NAME (must match <world name=...>)

    ros2 launch sink_sim sim.launch.py
    ros2 launch sink_sim sim.launch.py x:=0.0 y:=1.0 yaw:=0.0 rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

WORLD_NAME = 'hospital_sink'


def generate_launch_description():
    pkg = get_package_share_directory('sink_sim')
    cp_gz = get_package_share_directory('clearpath_gz')
    world_stem = os.path.join(pkg, 'worlds', WORLD_NAME)  # gz_sim adds '.sdf'

    args = [
        DeclareLaunchArgument('setup_path', default_value=[EnvironmentVariable('HOME'), '/clearpath/']),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # default spawn: ~1.6 m in front of the cabinet, a bit off-centre and turned,
        # so the aligner has real work to do
        DeclareLaunchArgument('x', default_value='1.4'),
        DeclareLaunchArgument('y', default_value='-0.35'),
        DeclareLaunchArgument('yaw', default_value='0.25'),
        DeclareLaunchArgument('z', default_value='0.3'),
    ]

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cp_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[
            ('world', world_stem),
            ('setup_path', LaunchConfiguration('setup_path')),
        ])

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cp_gz, 'launch', 'robot_spawn.launch.py')),
        launch_arguments=[
            ('world', WORLD_NAME),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ])

    return LaunchDescription(args + [gz, spawn])
