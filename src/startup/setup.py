from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'startup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf', 'robot', 'urdf'), glob('urdf/robot/urdf/*')),
        (os.path.join('share', package_name, 'urdf', 'robot', 'assets'), glob('urdf/robot/assets/*')),
        (os.path.join('share', package_name, 'urdf'), ['urdf/scene_info.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Surya Selvam',
    maintainer_email='surya.selvam03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
