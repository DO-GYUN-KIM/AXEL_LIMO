from setuptools import setup

package_name = 'autorace_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autorace team',
    maintainer_email='team@example.com',
    description='Lane, YOLO and LiDAR perception nodes for Autorace 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_detector = autorace_perception.lane_detector:main',
            'yolo_detector = autorace_perception.yolo_detector:main',
            'lidar_processor = autorace_perception.lidar_processor:main',
        ],
    },
)
