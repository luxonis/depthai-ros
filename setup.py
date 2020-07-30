from setuptools import setup

package_name = 'depthai_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jngai',
    maintainer_email='jon.ngai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = depthai_wrapper.publisher_member_function:main',
            'demoListen = depthai_wrapper.subscriber_member_function:main',
        ],
    },
)
