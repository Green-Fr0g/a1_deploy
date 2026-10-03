cmake_minimum_required(VERSION 3.8)
project(a1_mjcf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes mjcf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
