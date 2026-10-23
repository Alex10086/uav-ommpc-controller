from setuptools import setup

package_name = 'uav_ommpc_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xerolab',
    maintainer_email='xerolab@localhost',
    description='Minimal ROS 2 body-rate and thrust bridge to PX4.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['px4_bridge = uav_ommpc_ros.px4_bridge:main']},
)
