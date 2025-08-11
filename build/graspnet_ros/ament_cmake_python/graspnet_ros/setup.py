from setuptools import find_packages
from setuptools import setup

setup(
    name='graspnet_ros',
    version='0.0.1',
    packages=find_packages(
        include=('graspnet_ros', 'graspnet_ros.*')),
)
