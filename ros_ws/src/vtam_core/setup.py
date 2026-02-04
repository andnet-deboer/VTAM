from setuptools import setup
import os
from glob import glob

package_name = 'vtam_core'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install rviz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andnet DeBoer',
    maintainer_email='todo@todo.com',
    description='VTAM - Visuo-Tactile Assistive Manipulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'eflesh_node = vtam_core.eflesh_node:main',
            'viz_node = vtam_core.viz_node:main',
            'umi_gripper_node = vtam_core.umi_gripper_node:main',
            'eflesh_debug_node = vtam_core.eflesh_debug_node:main',
            'umi_pose_tracker = vtam_core.umi_pose_tracker:main',
            'umi_node = vtam_core.umi_node:main',
            'record_node = vtam_core.record_node:main'
        ],
    },
)
