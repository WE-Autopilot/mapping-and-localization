from setuptools import find_packages, setup

package_name = 'mapping_localization_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='ali@todo.todo',
    description='Mapping and Lanelet processing nodes',
    license='TODO',
    entry_points={
        'console_scripts': [
            'ap1_mapping_Lanelet = mapping_localization_python.ap1_mapping_lanelet:main',
            'test_osm_publisher = mapping_localization_python.test_osm_publisher:main',
            'test_lane_waypoints_publisher = mapping_localization_python.test_lane_waypoints_publisher:main',
        ],
    },
)

