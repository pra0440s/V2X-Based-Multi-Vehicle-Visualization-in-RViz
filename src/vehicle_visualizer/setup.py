from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vehicle_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*')),  # Install models
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranav',
    maintainer_email='pranav@todo.todo',
    description='Vehicle visualization package for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualize_cam = vehicle_visualizer.visualize_cam:main',  # Ensure your node is listed
        ],
    },
)

