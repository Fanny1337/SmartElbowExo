from setuptools import setup, find_packages
import os

package_name = 'serial_communication_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Fanny och Sophie',
    maintainer_email='fannyti@chalmers.se',
    description='Example package for handling serial communication in ROS 2 using Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_python_node = serial_communication_package.scripts.my_python_node:main'
        ],
    },
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
        [os.path.join('resource', package_name)])

    ],
)
