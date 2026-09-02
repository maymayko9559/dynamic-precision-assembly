from setuptools import find_packages, setup
import os
from glob import glob  

package_name = 'object_tracking'

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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_node = object_tracking.tracking_node:main',
            'kalman_filter = object_tracking.kalman_filter:main',
            'motion_predictor = object_tracking.motion_predictor:main',
            'velocity_estimator = object_tracking.velocity_estimator:main',
        ],
    },
)