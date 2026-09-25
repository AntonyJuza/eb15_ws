import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'eb15_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Antony Juza',
    maintainer_email='antonyjuza@gmail.com',
    description='ROS 2 serial bridge driver for EB-15 robotic arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eb15_serial_node = eb15_driver.eb15_serial_node:main',
            'unity_bridge_node = eb15_driver.unity_bridge_node:main',
        ],
    },
)
