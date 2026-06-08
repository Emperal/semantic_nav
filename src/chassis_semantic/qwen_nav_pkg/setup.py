import os
from setuptools import find_packages, setup

package_name = 'qwen_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    install_requires=[
        'setuptools'

    ],
    zip_safe=True,
    maintainer='Huangzifeng',
    maintainer_email='jetson@todo.todo',
    description='use orbbec, qwen and sam3 for items identify',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'qwen_nav_node = qwen_nav_pkg.qwen_nav_node:main'
        ],
    },
    data_files=[
        (os.path.join('share',package_name,'launch'),['launch/launch_qwen_sam3.py']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
)
