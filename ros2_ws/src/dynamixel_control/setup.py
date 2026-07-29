from glob import glob

from setuptools import find_packages, setup

package_name = 'dynamixel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_node = dynamixel_control.dynamixel_position_node:main',
            'dynamixel_position_node = dynamixel_control.dynamixel_position_node:main',
            'moveit_dynamixel_bridge = dynamixel_control.moveit_dynamixel_bridge:main',
            'yolo_bridge = dynamixel_control.yolo_to_dynamixel_bridge:main',
            'yolo_detection = dynamixel_control.yolo_detection_node:main',
            'arm_fsm = dynamixel_control.arm_fsm_node:main',
            'teleop_core = dynamixel_control.teleop_core_node:main',
            'keyboard_teleop = dynamixel_control.keyboard_teleop_node:main',
            'joystick_teleop = dynamixel_control.joystick_teleop_node:main',
        ],
    },
)
