from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tele_op'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rushi',
    maintainer_email='rdeshmukh@wpi.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = tele_op.keyboard_teleop:main',
            'dualsense_teleop = tele_op.dualsense_teleop:main',
        ],
    },
)
