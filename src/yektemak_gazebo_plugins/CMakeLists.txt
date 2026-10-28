cmake_minimum_required(VERSION 3.10)
project(yektemak_gazebo_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(yektemak_pallet_attacher SHARED src/pallet_attacher.cpp)
ament_target_dependencies(yektemak_pallet_attacher
  gazebo_dev gazebo_ros rclcpp std_msgs std_srvs)

install(TARGETS yektemak_pallet_attacher
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
