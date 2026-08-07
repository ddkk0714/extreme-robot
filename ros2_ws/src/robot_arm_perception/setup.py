from setuptools import find_packages, setup

package_name = 'robot_arm_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jo',
    maintainer_email='ddkk0714@naver.com',
    description='RealSense + YOLO perception node',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'perception_node = robot_arm_perception.perception_node:main',
            'stream_node = robot_arm_perception.stream_node:main',
            'metadata_sender_node = robot_arm_perception.metadata_sender_node:main',
            'vision_test = robot_arm_perception.vision_test_node:main',
        ],
    },
)
