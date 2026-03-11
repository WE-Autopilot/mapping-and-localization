import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mapping_localization_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description=(
        'Pipeline nodes for stabilizing perception data and bridging '
        'localization outputs.'
    ),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'perception_pipeline_node = '
                'mapping_localization_python.perception_pipeline_node:main'
            ),
            (
                'synthetic_perception_publisher_node = '
                'mapping_localization_python.'
                'synthetic_perception_publisher_node:main'
            ),
            (
                'slam_bridge_node = '
                'mapping_localization_python.slam_bridge_node:main'
            ),
            (
                'stored_point_registry_node = '
                'mapping_localization_python.stored_point_registry_node:main'
            ),
        ],
    },
)
