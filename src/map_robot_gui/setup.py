import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'map_robot_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pygame', 'numpy', 'Pillow'],
    zip_safe=True,
    maintainer='rm',
    maintainer_email='rm@example.com',
    description='Map drag-robot GUI for SP Nav tutorial simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_drag_robot = map_robot_gui.map_drag_robot_node:main',
        ],
    },
)
