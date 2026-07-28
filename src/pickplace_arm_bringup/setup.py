import os
from glob import glob
from setuptools import setup

package_name = 'pickplace_arm_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maedeh Jeddi',
    maintainer_email='maedehjeddi1993@gmail.com',
    description='Pick and place automation for pickplace_arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # The pick-and-place mission is the only behaviour with a launch
            # file, so it is the only behaviour with an executable. The modules
            # behind it are NOT dead code and must not be deleted: the mission
            # class is the end of an inheritance chain --
            #   PickAndPlace -> SearchAndPick -> NavAndPick -> Mission
            #                -> Mission2 -> Mission2Tugbot
            # so pick_and_place.py, search_and_pick.py, nav_and_pick.py,
            # mission.py and mission_2.py are all still imported at runtime.
            # They simply no longer have entry points of their own.
            'mission_pickPlace = pickplace_arm_bringup.mission_2:main_tugbot',
            # Manual driving, used when building a map with mapping.launch.py.
            'teleop_key = pickplace_arm_bringup.teleop_key:main',
            # Startup readiness gate used by mission_pickPlace.launch.py.
            'wait_for = pickplace_arm_bringup.wait_for:main',
        ],
    },
)
