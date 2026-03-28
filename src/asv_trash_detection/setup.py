from setuptools import setup

package_name = 'asv_trash_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='harish',
    description='Trash Detection Node',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.12/site-packages/asv_trash_detection/models', [
            'asv_trash_detection/models/trash_seg.pt'
        ]),
    ],
    entry_points={
        'console_scripts': [
            'trash_detection_node = asv_trash_detection.detection_node:main',
        ],
    },
)
