"""AprilTag detection on the front camera + the sink aligner.

    ros2 launch sink_sim align.launch.py
    ros2 launch sink_sim align.launch.py autostart:=false   # then call ~/start
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def clearpath_has_optical_frames():
    """True if the installed Clearpath RealSense model adds optical frames in sim."""
    try:
        path = os.path.join(get_package_share_directory('clearpath_sensors_description'),
                            'urdf', 'intel_realsense.urdf.xacro')
        with open(path) as f:
            return 'use_nominal_extrinsics' in f.read()
    except Exception:  # noqa: BLE001
        return False


def generate_launch_description():
    pkg = get_package_share_directory('sink_sim')
    ns = LaunchConfiguration('namespace')
    cam = LaunchConfiguration('camera')
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    # Clearpath publishes TF on <namespace>/tf, not /tf
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    image = ['sensors/', cam, '/color/image']
    info = ['sensors/', cam, '/color/camera_info']

    apriltag = Node(
        package='apriltag_ros', executable='apriltag_node', name='apriltag',
        namespace=ns, output='screen',
        parameters=[os.path.join(pkg, 'config', 'apriltag.yaml'), sim],
        remappings=tf_remaps + [('image_rect', image), ('camera_info', info)])

    # Older Clearpath releases (before 2.9.19) left the RealSense optical frames
    # out of the simulated robot, so the tag poses couldn't be linked to the robot.
    # On those, link <camera>_color_optical_frame to <camera>_link here. Newer
    # releases include the frames, and publishing them twice would break TF, so
    # this only runs when the installed Clearpath version needs it.
    # On the real robot the camera driver publishes them: use sim_optical_frame:=false.
    optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_optical_tf', namespace=ns, output='screen',
        condition=IfCondition(LaunchConfiguration('sim_optical_frame')),
        parameters=[sim],
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                   '--frame-id', [cam, '_link'],
                   '--child-frame-id', [cam, '_color_optical_frame']],
        remappings=tf_remaps)

    aligner = Node(
        package='sink_sim', executable='sink_aligner', name='sink_aligner',
        namespace=ns, output='screen',
        parameters=[os.path.join(pkg, 'config', 'aligner.yaml'), sim,
                    {'autostart': LaunchConfiguration('autostart')}],
        remappings=tf_remaps)

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r100_0000'),
        DeclareLaunchArgument('camera', default_value='camera_0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('sim_optical_frame',
                              default_value='false' if clearpath_has_optical_frames() else 'true'),
        optical_tf,
        apriltag,
        aligner,
    ])
