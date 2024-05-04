import glob
import os

from setuptools import setup

package_name = 'track_racing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/track_racing/launch', glob.glob(os.path.join('launch', '*launch.*'))),
        ('share/track_racing/map', glob.glob(os.path.join('map', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='yohang@mit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_simulator = track_racing.track_simulator:main'
        ],
    },
)
