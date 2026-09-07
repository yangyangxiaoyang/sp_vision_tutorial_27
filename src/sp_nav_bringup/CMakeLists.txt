cmake_minimum_required(VERSION 3.8)
project(sp_nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
