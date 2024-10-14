import os
import glob
from setuptools import setup

package_name = 'ntrip_client'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml', *glob.glob('launch/*')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Austin Johnson',
    maintainer_email='austin.johnson@anellophotonics.com',
    keywords=['ROS'],
    description='The ntrip client for the anello_ros_driver package',
    license='MIT License',
    entry_points={
        'console_scripts': [
            'ntrip_ros.py = ntrip_client.ntrip_ros:main',
        ],
    },
)