from setuptools import find_packages, setup

package_name = 'camera_pkg'

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
    maintainer='jidnyesha',
    maintainer_email='jpatil@wpi.edu',
    description='TODO: Package description',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rgbd_pub=camera_pkg.rgbd_publisher:main",
            "rgbd_sub=camera_pkg.rgbd_subscriber:main"
        ],
    },
)
