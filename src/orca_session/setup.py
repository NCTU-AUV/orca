from setuptools import setup

package_name = 'orca_session'

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
    description='Session logging helpers: current session, log directories, markers, '
                'FC parameter snapshot, bag slicing',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'marker_log = orca_session.marker_log:main',
        'mark = orca_session.mark:main',
        'fc_params = orca_session.fc_params:main',
        'bag_slice = orca_session.bag_slice:main',
    ]},
)
