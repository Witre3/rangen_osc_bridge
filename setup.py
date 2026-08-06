from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rangen_osc_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'python-osc'],
    # Reading a bag directly needs more than the robot does.  Kept as an extra
    # so the robot-side colcon install stays lean, and deliberately free of
    # rclpy (not on PyPI) so `pip install .[mcap]` works on a Mac with no ROS2.
    extras_require={
        'mcap': ['mcap', 'mcap-ros2-support', 'foxglove-sdk'],
    },
    zip_safe=True,
    maintainer='rangen',
    maintainer_email='tremblay.william@gmail.com',
    description='OSC bridge for end-effector motion → musical parameter mapping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ee_osc_bridge = rangen_osc_bridge.ee_osc_bridge:main',
            'state_interpreter = rangen_osc_bridge.state_interpreter:main',
            'mcap_osc_player = rangen_osc_bridge.mcap_osc_player:main',
        ],
    },
)
