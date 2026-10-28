from setuptools import setup

package_name = 'autorace_obstacle'

setup(
    name=package_name,
    version='0.1.0',
    packages=['autorace_obstacle'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autorace 2026 Team',
    maintainer_email='team@example.com',
    description='2D 라이다 장애물·라바콘 처리',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_node = autorace_obstacle.obstacle_node:main',
        ],
    },
)
