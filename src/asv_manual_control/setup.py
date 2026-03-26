


















from setuptools import find_packages, setup

package_name = 'asv_manual_control'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
	entry_points={
    	'console_scripts': [
        'thruster_node = asv_manual_control.thruster_node:main',
        'joy_thruster_node = asv_manual_control.joy_thruster_node:main',
    ],
},
)
