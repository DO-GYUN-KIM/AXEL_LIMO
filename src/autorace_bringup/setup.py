from glob import glob

from setuptools import setup

package_name = 'autorace_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autorace team',
    maintainer_email='team@example.com',
    description='Launch files and configs for Autorace 2026',
    license='MIT',
    entry_points={'console_scripts': []},
)
