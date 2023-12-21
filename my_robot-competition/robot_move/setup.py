from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_move'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('*.pt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sear',
    maintainer_email='vasilijkrukovskij2015@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'pid_reg = robot_move.pid_reg:main',
        		     'detect_line = robot_move.detect_line:main',
        		     'detector_sign = robot_move.detector_sign:main'
        ],
    },
)
