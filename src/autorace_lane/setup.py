from setuptools import setup

package_name = 'autorace_lane'

setup(
    name=package_name,
    version='0.1.0',
    packages=['autorace_lane'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autorace 2026 Team',
    maintainer_email='team@example.com',
    description='BEV 차선 인식 + Pure Pursuit 차선 추종 (D-Racer 이식 초안), BEV 캘리브레이션·오프라인 리플레이 도구',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_node = autorace_lane.lane_node:main',
            'bev_calib = autorace_lane.bev_calib:main',
            'offline_replay = autorace_lane.offline_replay:main',
            'lane_eval = autorace_lane.lane_eval:main',
        ],
    },
)
