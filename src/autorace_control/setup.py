from setuptools import setup

package_name = 'autorace_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.missions'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autorace team',
    maintainer_email='team@example.com',
    description='Lane following and mission state machine for Autorace 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_follower = autorace_control.lane_follower:main',
            'mission_manager = autorace_control.mission_manager:main',
            'cmd_vel_to_ackermann = autorace_control.cmd_vel_to_ackermann:main',
        ],
    },
)
