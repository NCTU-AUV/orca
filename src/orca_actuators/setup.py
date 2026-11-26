from glob import glob

from setuptools import setup

package_name = 'orca_actuators'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NCTU-AUV',
    description='Actuators on the Jetson: ball dropper electromagnet on a 40-pin GPIO',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'ball_dropper = orca_actuators.ball_dropper:main',
    ]},
)
