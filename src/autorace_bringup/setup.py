import os
from glob import glob

from setuptools import setup

package_name = 'autorace_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=['autorace_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autorace 2026 Team',
    maintainer_email='team@example.com',
    description='실행 파일(launch), 파라미터, 캘리브레이션 yaml, 점검 스크립트',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exposure_node = autorace_bringup.exposure_node:main',   # 11/19 출발 전 카메라 노출 고정
            'cam_level = autorace_bringup.cam_level:main',           # 11/19 노출 단계 저장/확인
        ],
    },
)
