from setuptools import find_packages
from skbuild import setup


package_name = 'apriltag_pose_estimation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*'], exclude=['*.testcase']),
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
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
    python_requires='>=3.10',
    zip_safe=False,
    author='MARS @ UVA',
    description='A Python library for pose estimation using AprilTags.'
)
