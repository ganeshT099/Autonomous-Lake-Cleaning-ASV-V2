from setuptools import setup, find_packages
from glob import glob

package_name = 'asv_trash_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harish',
    description='Trash Detection Node',
    license='MIT',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.pt')),
    ],

    entry_points={
        'console_scripts': [
            'trash_detection_node = asv_trash_detection.detection_node:main',
        ],
    },
)
