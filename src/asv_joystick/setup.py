from setuptools import setup

package_name = 'asv_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alcsv',
    maintainer_email='alcsv@todo.todo',
    description='ASV Joystick Control',
    license='TODO',
    entry_points={
        'console_scripts': [
            'asv_joystick_node = asv_joystick.asv_joystick_node:main',
        ],
    },
)
