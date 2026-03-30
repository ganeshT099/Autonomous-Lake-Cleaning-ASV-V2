from setuptools import setup
import os
from glob import glob
package_name = 'asv_camera'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASV Team',
    maintainer_email='gt0908@srmist.edu.in',
    description='Camera node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = asv_camera.camera_node:main',
        ],
    },
)
