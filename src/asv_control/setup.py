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
        ],
    },
)
