from setuptools import setup
import os
from glob import glob

package_name = 'asv_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harish',
    description='ASV Web System',
    license='MIT',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],

    entry_points={
        'console_scripts': [
            'camera_server = asv_web.camera_server:main',
            'ws_bridge = asv_web.ws_bridge:main',
        ],
    },
)
