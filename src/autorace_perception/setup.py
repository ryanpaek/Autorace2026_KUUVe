from setuptools import setup

package_name = 'autorace_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=['autorace_perception'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autorace 2026 Team',
    maintainer_email='team@example.com',
    description='YOLO 표지판·신호등 인식, 학습 이미지 수집기',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = autorace_perception.detector_node:main',
            'class_command_node = autorace_perception.class_command_node:main',
            'image_collector = autorace_perception.image_collector:main',
        ],
    },
)
