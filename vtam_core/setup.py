from setuptools import setup, find_packages 
import os
from glob import glob

package_name = 'vtam_core'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.xml') + glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andnet DeBoer',
    maintainer_email='deboerandnet@gmail.com',
    description='VTAM - Visuo-Tactile Assistive Manipulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vtam_robot_node = vtam_core.nodes.robot_node:main',
            'main_node = vtam_core.main_node:main',
            'record_node = vtam_core.nodes.record_node:main',
            'eflesh_node = vtam_core.drivers.eflesh_node:main',
            'umi_detector_node = vtam_core.drivers.umi_detector_node:main',
            'umi_gripper_node = vtam_core.nodes.umi_gripper_node:main',
            'cv_tracker = vtam_core.cv_tracker:main',
            'imu_calibration_helper = vtam_core.nodes.imu_helper:main',

            # Debug nodes
            'tactile_viz_node = vtam_core.debug.viz_node:main',
            'eflesh_debug_node = vtam_core.debug.eflesh_debug_node:main',
        ],
    },
)