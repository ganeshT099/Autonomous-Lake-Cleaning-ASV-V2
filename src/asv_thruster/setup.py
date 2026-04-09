from setuptools import setup

package_name = 'asv_thruster'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alcsv',
    maintainer_email='alcsv@todo.todo',
    description='ASV Thruster Control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'asv_thruster_node = asv_thruster.asv_thruster_node:main',
        ],
    },
)
