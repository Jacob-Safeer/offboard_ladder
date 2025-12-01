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
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jacob',
    maintainer_email='jsafeer1@terpmail.umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'velocity_control = px4_offboard.velocity_control:main',
                'control = px4_offboard.control:main',
                'circle_offboard = px4_offboard.circle_offboard_mode:main',
                'offboard_figure8 = px4_offboard.offboard_figure8_node:main',
                'offboard_ship_land = px4_offboard.offboard_ship_land:main',
                'offboard_ship_land_sim = px4_offboard.offboard_ship_land_sim:main',
                'offboard_test = px4_offboard.offboard_test:main',
                'land_test = px4_offboard.land_test:main',
        ],
    },
)
