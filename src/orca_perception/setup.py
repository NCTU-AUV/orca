from setuptools import setup

package_name = 'orca_perception'

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
    description='Object detection node (RF-DETR)',
    license='Apache-2.0',
    entry_points={'console_scripts': ['detector = orca_perception.detector:main']},
)
