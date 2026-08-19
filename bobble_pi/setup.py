import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bobble_pi'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BobbleBot',
    maintainer_email='sharina22calsin@gmail.com',
    description=(
        'Raspberry Pi 4 hardware bring-up for the BobbleBot self-balancing '
        'robot (ROS 2 Jazzy): MPU6050 IMU, L298N + JGY-370 motor driver, '
        'and the cascade PID balance controller.'
    ),
    license='tbd',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = bobble_pi.imu_node:main',
            'imu_calibrate = bobble_pi.imu_calibrate:main',
            'balance_controller_node = bobble_pi.balance_controller_node:main',
            'keyboard_control_node = bobble_pi.keyboard_control_node:main',
            'joystick_control_node = bobble_pi.joystick_control_node:main',
        ],
    },
)
