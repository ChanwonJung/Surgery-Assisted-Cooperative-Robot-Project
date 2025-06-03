from setuptools import find_packages, setup
import glob
import os

package_name = 'pick_and_place_voice'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(include=[
        'robot_control', 
        'voice_processing', 
        'object_detection',
        'detect_wound'
    ]),

    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*')),
        ('share/' + package_name + '/resource', glob.glob('resource/.env')),
        # ('share/ament_index/resource_index/packages',['resource/' + 'voice_processing']),
        # ('share/voice_processing', ['package.xml']),
        # ('share/object_detection', ['package.xml']),
        # ('share/robot_control', ['package.xml']),

        # ('share/' + package_name + '/launch', glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey4090',
    maintainer_email='rokey4090@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'robot_control = robot_control.robot_control:main',
            'object_detection = object_detection.detection:main',
            'get_keyword = voice_processing.get_keyword:main',
            'detect_wound = detect_wound.detect_wound:main'
        ],
    },
)