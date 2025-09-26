from setuptools import find_packages, setup

package_name = 'localization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localization_launch.py']),  # Add launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarek_22',
    maintainer_email='tarek_22@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = localization_pkg.localization_node:main'
        ],
    },
)
