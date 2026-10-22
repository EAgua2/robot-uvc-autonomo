import os
from glob import glob
from setuptools import setup

package_name = 'uvc_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jesus Eduardo Agua Cervantes',
    description='Cobertura UV-C: planificador CPP + A*, secuenciador Nav2 y puentes con ESP32.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'coverage_node = uvc_robot.coverage_node:main',
            'goal_pose_sequencer = uvc_robot.goal_pose_sequencer:main',
            'serial_bridge = uvc_robot.serial_node:main',
            'esp32_lamp_node = uvc_robot.esp32_lamp_node:main',
        ],
    },
)
