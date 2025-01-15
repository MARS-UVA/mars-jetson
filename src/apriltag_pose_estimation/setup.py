from setuptools import setup, find_packages


package_name = 'apriltag_pose_estimation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pupil-apriltags',
        'scipy'
    ],
    zip_safe=True,
    author='MARS @ UVA',
    description='A Python library for estimating the pose of AprilTags.'
)
