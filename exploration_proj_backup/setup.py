from setuptools import find_packages
from setuptools import setup

package_name = 'exploration_project'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=[]),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    description='TurtleBot3 autonomous exploration project',

    license='Apache License, Version 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'frontier_detection = exploration_project.frontier_detection:main',
            'frontier_selector = exploration_proj.frontier_selector:main',
            #sensorcode
        ],
    },
)