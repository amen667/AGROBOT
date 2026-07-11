import os
from setuptools import find_packages, setup

package_name = 'agrobot_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/lidar_view.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/lidar_view.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amen',
    maintainer_email='amenallahbenassi@gmail.com',
    description='Basic ROS2 nodes for the AGROBOT project',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'status_publisher = agrobot_basics.status_publisher:main',
        ],
    },
)