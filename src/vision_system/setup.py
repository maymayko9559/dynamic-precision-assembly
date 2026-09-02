from setuptools import find_packages, setup
import os
from glob import glob  

package_name = 'vision_system'

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
            'vision_manager = vision_system.vision_manager:main',
            'coordinate_transform = vision_system.coordinate_transform:main',
            'image_processing = vision_system.image_processing:main',
            'shape_detector = vision_system.shape_detector:main',
        ],
    },
)
