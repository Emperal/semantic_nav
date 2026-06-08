import os
from setuptools import find_packages, setup

package_name = 'bson_rpc'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=[
        'setuptools',
        'bsonrpc2',   # 如果是 bsonrpc，需要安装
        'rclpy',
        'nav2_msgs',
        # 'tf-transformations',
        'std_msgs'
    ],
    zip_safe=True,
    maintainer='Huangzifeng',
    maintainer_email='youremail@example.com',
    description='ROS2 Python node with BSON-RPC server for Nav2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rpc_server_node = bson_rpc.rpc_server:main',
        ],
    },
    data_files=[
        (os.path.join('share',package_name,'launch'),['launch/rpc_server_launch.py']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
)