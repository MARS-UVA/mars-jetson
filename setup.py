from setuptools import setup, find_packages


package_name = 'apriltag_pose_estimation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*'], exclude=['*.testcase']),
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pupil_apriltags',
        'scipy'
    ],
    extras_require={
        'render': [
            'meshio',
            'pyvista',
            'pyvistaqt',
            'PyQt5'
        ],
        'generate': [
            'Pillow',
            'fpdf2'
        ],
        'complete': [
            'meshio',
            'pyvista',
            'pyvistaqt',
            'PyQt5',
            'Pillow',
            'fpdf2'
        ]
    },
    zip_safe=False,
    author='MARS @ UVA',
    description='A Python library for pose estimation using AprilTags.'
)
