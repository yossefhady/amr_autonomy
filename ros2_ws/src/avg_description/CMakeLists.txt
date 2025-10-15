cmake_minimum_required(VERSION 3.8)
project(avg_description)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf worlds launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
