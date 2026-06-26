from setuptools import setup
import os
from glob import glob

package_name = 'rc_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bhivesh',
    maintainer_email='bhivesh@abstractlabs.dev',
    description='Pi-side nodes for the RC car: ESP32 bridge, IMU, controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = rc_car.esp32_bridge:main',
            'imu_node = rc_car.imu_node:main',
            'controller_node = rc_car.controller_node:main',
            'maneuver_client = rc_car.maneuver_client:main',
        ],
    },
)
