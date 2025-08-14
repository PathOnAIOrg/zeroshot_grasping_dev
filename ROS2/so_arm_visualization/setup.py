from setuptools import find_packages, setup

package_name = 'so_arm_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'so-101-rviz.png', 'so-101-rviz.gif']),
        ('share/' + package_name + '/launch', ['launch/so_arm_visualization.launch.py']),
        # URDF files
        ('share/' + package_name + '/urdf', [
            'urdf/so101.urdf.xacro',
            'urdf/so101_base.xacro', 
            'urdf/so101_gazebo.xacro',
            'urdf/so101_ros2_control.xacro'
        ]),
        # RViz configuration
        ('share/' + package_name + '/rviz', ['rviz/display.rviz']),
        # Mesh files
        ('share/' + package_name + '/meshes/so101', [
            'meshes/so101/base_motor_holder_so101_v1.stl',
            'meshes/so101/base_so101_v2.stl',
            'meshes/so101/motor_holder_so101_base_v1.stl',
            'meshes/so101/motor_holder_so101_wrist_v1.stl',
            'meshes/so101/moving_jaw_so101_v1.stl',
            'meshes/so101/rotation_pitch_so101_v1.stl',
            'meshes/so101/sts3215_03a_no_horn_v1.stl',
            'meshes/so101/sts3215_03a_v1.stl',
            'meshes/so101/under_arm_so101_v1.stl',
            'meshes/so101/upper_arm_so101_v1.stl',
            'meshes/so101/waveshare_mounting_plate_so101_v2.stl',
            'meshes/so101/wrist_roll_follower_so101_v1.stl',
            'meshes/so101/wrist_roll_pitch_so101_v2.stl'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shyam',
    maintainer_email='ganatras@asu.edu',
    description='ROS2 package for SO-ARM101 robot visualization with real hardware interface',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'so_arm_joint_reader = so_arm_visualization.so_arm_joint_reader:main',
        ],
    },
)
