from setuptools import setup

package_name = 'inventory_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspberry',
    maintainer_email='raspberry@todo.todo',
    description='Inventory robot ROS 2 package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge    = inventory_robot.serial_bridge:main',
            'obstacle_avoid   = inventory_robot.obstacle_avoid:main',
            'camera_scan_node = inventory_robot.camera_scan_node:main',
        ],
    },
)
