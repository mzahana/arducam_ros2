from setuptools import setup

package_name = 'arducam_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
