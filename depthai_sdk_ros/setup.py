from setuptools import setup

package_name = 'depthai_sdk_ros'

setup(
    name=package_name,
    version='2.6.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
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
