cmake_minimum_required(VERSION 3.10)
project(yektemak_unitree_lidar)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Unitree L1 vendor SDK
set(
  UNITREE_SDK_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/unilidar_sdk/unitree_lidar_sdk"
)

if(NOT EXISTS "${UNITREE_SDK_DIR}/include/unitree_lidar_sdk.h")
  message(
    FATAL_ERROR
    "Unitree L1 SDK not found at ${UNITREE_SDK_DIR}"
  )
endif()

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${UNITREE_SDK_DIR}/include
)

add_definitions(${PCL_DEFINITIONS})

add_executable(
  evrenos_unitree_l1_udp_node
  src/evrenos_unitree_l1_udp_node.cpp
)

ament_target_dependencies(
  evrenos_unitree_l1_udp_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(
  evrenos_unitree_l1_udp_node
  ${PCL_LIBRARIES}
  "${UNITREE_SDK_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_lidar_sdk.a"
)

install(
  TARGETS
    evrenos_unitree_l1_udp_node
  DESTINATION
    lib/${PROJECT_NAME}
)
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)
ament_package()