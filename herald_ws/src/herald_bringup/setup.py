import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'herald_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'dashboard'), glob('dashboard/*.html')),
    ],
    install_requires=['setuptools', 'pyserial', 'smbus2'],
    zip_safe=True,
    maintainer='Keegan',
    maintainer_email='keegan@example.com',
    description='HERALD rover bringup: serial bridge, tilt compensation, floor-drop detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_bridge_node = herald_bringup.motor_bridge_node:main',
            'tilt_compensation_node = herald_bringup.tilt_compensation_node:main',
            'floor_drop_detector_node = herald_bringup.floor_drop_detector_node:main',
            'autonomy_manager_node = herald_bringup.autonomy_manager_node:main',
            'hazard_classification_node = herald_bringup.hazard_classification_node:main',
        ],
    },
)
