from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voice_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'resource'),
         glob('resource/*.tflite') + glob('resource/*.json') + glob('resource/.env')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='csh980625@gmail.com',
    description='voice command interface for dynamic precision assembly',
    license='TODO: License declaration',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'audio_device=voice_pkg.audio_device:main',
            'keyword_extraction = voice_pkg.keyword_extraction:main',
            'mic_test = voice_pkg.mic_test:main',
            'MicController = voice_pkg.MicController:main',
            'STT=voice_pkg.STT:main',
            'wakeup_word=voice_pkg.wakeup_word:main',
            'voice_command_node = voice_pkg.voice_command_node:main',
        ],
    },
)
