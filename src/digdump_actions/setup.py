from setuptools import find_packages, setup


setup(
    name='digdump_actions',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/digdump_actions']),
        ('share/digdump_actions', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MARS',
    maintainer_email='ericzn248@gmail.com',
    description='Separate Python dig and dump action servers with Bruno routines.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'dig_node = digdump_actions.dig:main',
        'dump_node = digdump_actions.dump:main',
    ]},
)
