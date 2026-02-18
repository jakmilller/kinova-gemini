from setuptools import find_packages, setup

package_name = 'gemini_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mcrr-lab',
    maintainer_email='jmill.pro1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemini_brain = gemini_robotics.gemini_brain_node:main',
            'text_interface = gemini_robotics.text_interface_node:main',
        ],
    },
)
