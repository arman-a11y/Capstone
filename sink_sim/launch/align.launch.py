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

    # Clearpath's RealSense model doesn't define the optical frames (the real
    # camera driver publishes them). In sim the image is stamped with
    # <camera>_color_optical_frame, so link it to <camera>_link here.
    # Only needed in simulation: launch with sim_optical_frame:=false on the robot.
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
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        DeclareLaunchArgument('camera', default_value='camera_0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('sim_optical_frame', default_value='true'),
        optical_tf,
        apriltag,
        aligner,
    ])
