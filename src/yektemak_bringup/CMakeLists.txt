cmake_minimum_required(VERSION 3.10)
project(yektemak_bringup)

find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------
# Real vehicle launch/config files
# ------------------------------------------------------------

install(
  DIRECTORY
    launch
    config
    docs
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------------------------
# Python executable nodes
# ------------------------------------------------------------

install(
  PROGRAMS
    scripts/evrenos_stm32_bridge.py
    scripts/evrenos_mock_wheel_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
