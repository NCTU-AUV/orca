from glob import glob

from setuptools import setup

package_name = 'orca_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name, ['scripts/bridge.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NCTU-AUV',
    description='AuvControl to ArduSub bridge (mavros + auv_adapter)',
    license='Apache-2.0',
    entry_points={'console_scripts': ['auv_adapter = orca_bridge.auv_adapter:main']},
)
