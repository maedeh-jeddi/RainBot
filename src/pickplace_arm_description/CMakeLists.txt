cmake_minimum_required(VERSION 3.8)
project(pickplace_arm_description)

find_package(ament_cmake REQUIRED)

# `meshes/` and `urdf/vendor/` hold VENDORED third-party geometry: the Husky
# A200 (Clearpath), the FR3 arm + Franka Hand (Franka Robotics), and the
# RealSense/SICK sensor housings. These used to be referenced out of their own
# packages via $(find ...) / package://, but those packages were nested git
# checkouts that could not be pushed with this project, so only what this robot
# actually uses has been copied in here. Upstream licences sit alongside the
# meshes as LICENSE.<package>.
# `fuel_cache/` is a vendored copy of the Gazebo Fuel models the Tugbot
# warehouse world pulls in (warehouse shell, shelves, pallets, carts, Tugbot).
# Pointing GZ_FUEL_CACHE_PATH at it (see gazebo.launch.py) makes the world
# load with no network access and no ~/.gz/fuel pre-fetch step.
# `aws_hospital_models/` holds the building shell, curtains, elevator and
# nurses' station that ship with AWS RoboMaker's hospital world; aws_hospital.sdf
# keeps AWS's own model:// URIs for them and gazebo.launch.py puts this
# directory on GZ_SIM_RESOURCE_PATH. Leaving it out of this list does not fail
# the build - it fails at RUNTIME, and loudly: every one of those URIs comes back
# as "Error Code 14" against a path under install/, the world never finishes
# loading, and the gz server exits and takes the whole launch down with it.
# Amazon's licence sits alongside the meshes.
install(
  DIRECTORY urdf launch rviz config worlds models meshes fuel_cache
            aws_hospital_models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
