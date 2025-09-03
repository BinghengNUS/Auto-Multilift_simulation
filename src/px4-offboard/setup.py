import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'Reference_traj_fig8'),
         glob('px4_offboard/Reference_traj_fig8/*')),
        (os.path.join('share', package_name, 'trained data'),
         glob('px4_offboard/trained data/*')),
        (os.path.join('share', package_name), glob('px4_offboard/NeuralNet.py')),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunchao',
    maintainer_email='yunchao.li@u.nus.edu',
    description='PX4 Offboard Control Package for Auto-multilift',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'visualizer = px4_offboard.visualizer:main',

                # Geometric control
                'geom_multi = px4_offboard.geom_multi:main',
                'clock_sync_node = px4_offboard.clock_sync_node:main',
        ],
    },
)
