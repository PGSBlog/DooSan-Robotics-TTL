from setuptools import setup
import os
from glob import glob

package_name = 'table_service_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),
        (os.path.join('share', package_name, 'srv'),
         glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='droppgs',
    maintainer_email='droppgs@todo.todo',
    description='Table Service Robot Package',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'navigation_node = table_service_robot.navigation_node:main',
            'navigation_gui = table_service_robot.navigation_gui:main',
            'kiosk = table_service_robot.kiosk:main',
            'kitchen = table_service_robot.kitchen:main',
        ],
    },
)
