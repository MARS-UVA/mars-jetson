from setuptools import setup

setup(
    name='apriltag_pose_estimation',
    version='0.0.1',
    packages=['apriltag_pose_estimation'],
    url='',
    license='',
    author='MARS @ UVA',
    author_email='',
    description='A Python library for estimating the pose of AprilTags.',
    install_requires=[
        'numpy',
        'opencv-python',
        'pupil-apriltags'
    ]
)
