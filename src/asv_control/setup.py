from setuptools import find_packages, setup

package_name = 'asv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alcsv',
    maintainer_email='alcsv@todo.todo',
    description='ASV control node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_node = asv_control.control_node:main',
		'geofence_node = asv_control.geofence_node:main',
			'zigzag_planner_node = asv_control.zigzag_planner_node:main',
				'waypoint_follower_node = asv_control.waypoint_follower_node:main',
					'safety_node = asv_control.safety_node:main',
        ],
    },
)
