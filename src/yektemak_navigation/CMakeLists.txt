cmake_minimum_required(VERSION 3.10)

project(yektemak_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
    maps
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/evrenos_auto_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
