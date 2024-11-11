from setuptools import find_packages, setup

package_name = 'order_database'

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
    maintainer='shin',
    maintainer_email='ehddbs1211@naver.com',
    description='A ROS 2 package for managing restaurant orders with a SQLite database',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_database_node = order_database.order_database_node:main',
        ],
    },
)