import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'assignment1_rt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='belal',
    maintainer_email='belal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui_node = assignment1_rt.ui_node:main',
            'turtle_spawn = assignment1_rt.turtle_spawn:main',
            'distance_monitor = assignment1_rt.distance_monitor:main', # Added new node
        ],
    },
)
