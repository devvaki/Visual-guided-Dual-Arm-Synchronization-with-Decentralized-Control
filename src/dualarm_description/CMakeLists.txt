cmake_minimum_required(VERSION 3.8)
project(dualarm_description)

find_package(ament_cmake REQUIRED)
# xacro is executed at runtime; no find_package(xacro) needed here.

# Install the URDF/Xacro and (optionally) meshes/include
install(DIRECTORY urdf include
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
