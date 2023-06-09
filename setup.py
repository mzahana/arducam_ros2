from setuptools import setup
import os
from glob import glob

package_name = 'arducam_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('launch/*.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Abdelkader',
    maintainer_email='mohamedashraf123@gmail.com',
    description='This package interfaces Arducam cameras that support v4l2 to ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arducam_stereo = arducam_ros2.arducam_stereo:main',
        ],
    },
)
