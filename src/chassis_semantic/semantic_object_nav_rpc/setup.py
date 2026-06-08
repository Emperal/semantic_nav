from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'semantic_object_nav_rpc'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot2',
    maintainer_email='robot2@example.com',
    description='Read pyslam semantic NPZ map, retrieve semantic objects, and send navigation goals through bsonrpc.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_object_rpc_server = semantic_object_nav_rpc.semantic_object_rpc_server:main',
            'semantic_object_rpc_client = semantic_object_nav_rpc.semantic_object_rpc_client:main',
        ],
    },
)
