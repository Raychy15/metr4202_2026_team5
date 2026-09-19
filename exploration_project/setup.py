from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'exploration_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='Autonomous exploration and vision mapping',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_localizer = exploration_project.target_localiser:main',
            'frontier_detector = exploration_project.frontier_detection:main',
            'frontier_selector = exploration_project.frontier_selector:main'
        ],
    },
)