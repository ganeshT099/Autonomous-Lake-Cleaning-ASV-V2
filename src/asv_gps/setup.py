from setuptools import setup

package_name = 'asv_gps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alcsv',
    maintainer_email='your@email.com',
    description='ASV GPS Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gps_node = asv_gps.gps_node:main',
        ],
    },
)
