from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'joy_publisher'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),  # This will automatically include your packages and sub-packages
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maximilian Ehrhardt',
    maintainer_email='max.ehrhardt@hotmail.de',
    description='Core functionalities for the ExoMy rover',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_controller = joy_publisher.joy_controller:main',
            'joy_publisher = joy_publisher.joy_publisher:main',
            'joy_publisher2 = joy_publisher.joy_publisher2:main',
            'distance_calculator = joy_publisher.distance_calculator:main',
            'position_calculator = joy_publisher.position_calculator:main',
            'imu_publisher = joy_publisher.imu_publisher:main',
        ],
    },
)
