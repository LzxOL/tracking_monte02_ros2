from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Monte_api_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'tf2_ros'],
    zip_safe=True,
    maintainer='root1',
    maintainer_email='root1@todo.todo',
    description='Query and print TF relation between link_t0_base and link_h2_head via RobotLib',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_base_tf_node=Monte_api.head_base_tf_node:main',
        ],
    },
)

