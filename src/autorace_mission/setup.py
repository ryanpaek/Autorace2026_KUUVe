from setuptools import setup

package_name = 'autorace_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=['autorace_mission'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autorace 2026 Team',
    maintainer_email='team@example.com',
    description='미션 상태 머신 + 최종 /cmd_vel 선택기, 주차 노드 뼈대',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = autorace_mission.mission_node:main',
            'parking_node = autorace_mission.parking_node:main',
            'ackermann_bridge = autorace_mission.ackermann_bridge:main',
            'debug_view = autorace_mission.debug_view:main',
            'chime_node = autorace_mission.chime_node:main',
            'sys_monitor = autorace_mission.sys_monitor:main',
        ],
    },
)
