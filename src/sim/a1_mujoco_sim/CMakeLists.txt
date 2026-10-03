cmake_minimum_required(VERSION 3.8)
project(a1_mujoco_sim)
find_package(ament_cmake REQUIRED)

# Executable launcher and the Python package it imports.
install(PROGRAMS scripts/a1_mujoco_sim.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY a1_mujoco_sim
  DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
