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
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
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
            # Blood sample transport in the hospital world. Same Mission2
            # sequence relocated, so mission_hospital.py is layout plus payload
            # geometry, not a second implementation.
            'mission_hospital = pickplace_arm_bringup.mission_hospital:main',
            # Walks the hospital corridor's pedestrians up and down their lanes
            # so they are moving obstacles rather than scenery.
            'corridor_pedestrians = '
            'pickplace_arm_bringup.corridor_pedestrians:main',
            # Manual driving, used when building a map with mapping.launch.py.
            'teleop_key = pickplace_arm_bringup.teleop_key:main',
            # Automated replacement for teleop_key when mapping the hospital
            # world: drives a fixed, clearance-checked tour of the building so
            # maps/hospital_lab is reproducible rather than depending on how
            # someone happened to drive. hospital_route_check re-verifies the
            # tour against every prop footprint and must be run after any edit
            # to it -- the first tour drove straight into a standing figure
            # precisely because its waypoints were clear but a segment BETWEEN
            # two of them was not.
            'hospital_map_drive = '
            'pickplace_arm_bringup.hospital_map_drive:main',
            'hospital_route_check = '
            'pickplace_arm_bringup.hospital_route_check:main',
            # Proves every actor trajectory in aws_hospital.sdf clears the
            # walls, the furniture, the people standing about and each other.
            # Actors perceive nothing, so an unchecked path is walked THROUGH
            # whatever is on it.
            'hospital_actor_paths = '
            'pickplace_arm_bringup.hospital_actor_paths:main',
            # Startup readiness gate used by mission_pickPlace.launch.py.
            'wait_for = pickplace_arm_bringup.wait_for:main',
        ],
    },
)
