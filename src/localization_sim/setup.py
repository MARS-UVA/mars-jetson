import os
from glob import glob
from pathlib import Path
from setuptools import find_packages, setup


package_name = 'localization_sim'


def get_sim_data() -> list[tuple[str, str]]:
    root_source_dir = Path('sim_data')
    root_target_dir = Path('share', package_name, 'sim_data')
    return (
        (str(root_target_dir / subdir.name), [str(p) for p in subdir.iterdir()])
        for subdir in (p for p in root_source_dir.iterdir() if p.is_dir())
    )


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        *get_sim_data(),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosdev',
    maintainer_email='ivan.post24@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulated_data = localization_sim.simulated_data:main',
            'pose_listener = localization_sim.pose_listener:main',
        ],
    },
)
