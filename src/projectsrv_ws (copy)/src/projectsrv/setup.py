from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'projectsrv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'sounds'), glob('projectsrv/sounds/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shin',
    maintainer_email='ehddbs1211@naver.com',
    description='Kiosk ordering and kitchen display server for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kiosk = projectsrv.kiosk:main',
            'kitchen = projectsrv.kitchen:main',
        ],
    },
)

