from setuptools import setup

package_name = 'orca_cam_bridge'

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
    maintainer='NCTU-AUV',
    description='Receive JPEG stream from the RPi cam-relay and publish it like realsense2_camera',
    license='Apache-2.0',
    entry_points={'console_scripts': ['cam_bridge = orca_cam_bridge.cam_bridge:main']},
)
