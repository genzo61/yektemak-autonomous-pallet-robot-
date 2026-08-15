cmake_minimum_required(VERSION 3.10)

project(yektemak_gazebo)

find_package(ament_cmake REQUIRED)


# Launch, world, config ve RViz dosyaları
install(
  DIRECTORY
    launch
    worlds
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)


# Çalıştırılabilir Python node'ları
install(
  PROGRAMS
    scripts/evrenos_moving_obstacle.py
    scripts/evrenos_manual_drive.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


ament_package()