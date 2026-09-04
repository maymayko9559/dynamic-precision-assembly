from setuptools import find_packages, setup
import os
from glob import glob  

package_name = 'dynamic_assembly_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='may',
    maintainer_email='maymayko9559@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'assembly_controller = dynamic_assembly_robot_control.assembly_controller:main',
            'insertion_controller = dynamic_assembly_robot_control.insertion_controller:main',
            'motion_planner = dynamic_assembly_robot_control.motion_planner:main',
            'target_manager = dynamic_assembly_robot_control.target_manager:main',
            'motion_utils = dynamic_assembly_robot_control.motion_utils:main',
        ],
    },
)
