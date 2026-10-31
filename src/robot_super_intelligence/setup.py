from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_super_intelligence'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MARS',
    maintainer_email='mars@virginia.edu',
    description='ROS 2 action orchestration for autonomous excavation runs.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'state_manager = robot_super_intelligence.manager:main',
    ]},
)
