import os
from setuptools import setup, find_packages
from glob import glob

package_name = 'gemini_live_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcrr-lab',
    maintainer_email='jmill.pro1@gmail.com',
    description='Gemini Live API Manager/Worker Architecture for Kinova',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemini_live_brain_simple = gemini_live_robotics.gemini_live_brain_simple:main',
            'gemini_live_brain = gemini_live_robotics.gemini_live_brain_node:main'
        ],
    },
)
