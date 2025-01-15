from setuptools import setup, find_packages


package_name = 'apriltag_pose_estimation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pupil-apriltags',
        'scipy'
    ],
    zip_safe=True,
    author='MARS @ UVA',
    maintainer='rosdev',
    maintainer_email='ivan.post24@gmail.com',
    description='A Python library for estimating the pose of AprilTags.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
