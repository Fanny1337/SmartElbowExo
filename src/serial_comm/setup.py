#!/usr/bin/env python3
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'serial_comm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("lib", package_name), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fanny',
    maintainer_email='fannyti@chalmers.se',
    description='Package for controlling a servo via ROS2 and Arduino',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        "serial_comm_node = serial_comm.scripts.serial_comm_node:main",
        "pcarduino_node = serial_comm.scripts.pcarduino:main",
        "pcarduino_ros_node = serial_comm.scripts.pcarduino_ros:main",
        "pcarduino_ros_touch_node = serial_comm.scripts.pcarduino_ros_touch:main",
        "pcarduino_step_node = serial_comm.scripts.pcarduino_step:main",
        "pcarduino_to_traj_node = serial_comm.scripts.pcarduino_to_traj:main",
        "trajectory_1_node = serial_comm.scripts.trajectory_1:main",
    ],
}

)    
    
    #{
    #     'console_scripts': [
    #         "serial_comm_node = serial_comm.scripts.serial_comm_node:main",
    #         "pcarduino_node = serial_comm.scripts.pcarduino:main",
    #         "pcarduino_ros_node = serial_comm.scripts.pcarduino_ros:main",
    #         "pcarduino_ros_touch_node = serial_comm.scripts.pcarduino_ros_touch:main",
    #         "pcarduino_step_node = serial_comm.scripts.pcarduino_step:main",
    #         "pcarduino_to_traj_node = serial_comm.scripts.pcarduino_to_traj:main",
    #         "trajectory_1_node = serial_comm.scripts.trajectory_1:main",
    #     ],
    # },

