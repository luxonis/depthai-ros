from setuptools import setup

package_name = 'depthai_sdk_ros'
submodules = "depthai_sdk_ros/ros_integration"

setup(
    name=package_name,
    version='2.7.1',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'depthai_sdk'],
    zip_safe=True,
    maintainer='Adam Serafin',
    maintainer_email='adam.serafin@luxonis.com',
    description='Depthai SDK ROS driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = depthai_sdk_ros.camera:main'
        ],
    },
)
