from glob import glob

from setuptools import setup

package_name = 'orca_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.bt', package_name + '.tasks'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/trees', glob('trees/*.yaml') + glob('trees/*.md')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NCTU-AUV',
    description='Behaviour-tree mission layer (py_trees)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mission = orca_mission.node:main',
        'render_tree = orca_mission.bt.render:main',
    ]},
)
