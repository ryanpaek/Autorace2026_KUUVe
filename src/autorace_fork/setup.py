import os
from glob import glob
from setuptools import setup

package_name = 'autorace_fork'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autorace 2026 Team',
    maintainer_email='team@example.com',
    description='미션 2 갈림길 A/B 판단',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fork_node = autorace_fork.fork_node:main',
        ],
    },
)
