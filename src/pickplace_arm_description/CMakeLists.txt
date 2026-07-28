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
install(
  DIRECTORY urdf launch rviz config worlds models meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
