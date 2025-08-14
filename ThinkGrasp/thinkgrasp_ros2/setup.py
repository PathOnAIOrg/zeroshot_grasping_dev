from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'thinkgrasp_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ThinkGrasp Team',
    maintainer_email='thinkgrasp@example.com',
    description='ROS2 wrapper for ThinkGrasp',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thinkgrasp_service_node = thinkgrasp_ros2.thinkgrasp_service_node:main',
            'thinkgrasp_client_node = thinkgrasp_ros2.thinkgrasp_client_node:main',
            'simple_client_node = thinkgrasp_ros2.simple_client_node:main',
            'image_capture_node = thinkgrasp_ros2.image_capture_node:main',
            'grasp_visualizer_node = thinkgrasp_ros2.grasp_visualizer_node:main',
        ],
    },
)