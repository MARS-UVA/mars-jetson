from setuptools import find_packages, setup

package_name = 'serial_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mars',
    maintainer_email='ericzn248@gmail.com',
    description='Serial communications; reading from tele-op to send to jetson',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'op_reader = serial_node.node:main'
        ],
    },
)
