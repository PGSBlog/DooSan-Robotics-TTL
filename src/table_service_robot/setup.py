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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='droppgs',
    maintainer_email='droppgs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'navigation_node = table_service_robot.navigation_node:main',
            'delivery_client = table_service_robot.delivery_action_client:main',
            'navigation_gui = table_service_robot.navigation_gui:main' ,
        ],
    },
)
