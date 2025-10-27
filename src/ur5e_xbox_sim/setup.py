from setuptools import setup
import os
from glob import glob

package_name = 'ur5e_xbox_sim'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kiranraj',
    maintainer_email='you@example.com',
    description='UR5e Gazebo + Xbox teleop package',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'teleop_node = ur5e_xbox_sim.teleop_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
)
