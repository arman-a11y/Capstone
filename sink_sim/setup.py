from glob import glob

from setuptools import setup

package_name = 'sink_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/clearpath', glob('clearpath/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/worlds/textures', glob('worlds/textures/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sink drain robot team',
    maintainer_email='dev@example.com',
    description='Husky A200 + UR5 sink alignment simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sink_aligner = sink_sim.sink_aligner:main',
            'arm_pose = sink_sim.arm_pose:main',
        ],
    },
)
