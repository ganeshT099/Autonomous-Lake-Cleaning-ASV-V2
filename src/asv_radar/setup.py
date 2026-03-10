from setuptools import setup
import os
from glob import glob
package_name = 'asv_radar'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASV Team',
    maintainer_email='gt0908@srmist.edu.in',
    description='DFRobot C4001 Radar node for ASV',
    license='MIT',
    entry_points={
        'console_scripts': [
            'asv_radar_node = asv_radar.radar_node:main',
        ],
    },
)
