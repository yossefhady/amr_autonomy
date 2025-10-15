cmake_minimum_required(VERSION 3.8)
project(avg_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY msg srv
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
