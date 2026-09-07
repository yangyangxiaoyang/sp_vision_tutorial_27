cmake_minimum_required(VERSION 3.8)
project(sp_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(sp_global_planner_plugins SHARED
  plugins/astar_planner.cpp
)
ament_target_dependencies(sp_global_planner_plugins
  rclcpp nav_msgs geometry_msgs pluginlib tf2 tf2_geometry_msgs
)
pluginlib_export_plugin_description_file(sp_global_planner plugin_description.xml)

add_executable(planner_server
  src/main.cpp
  src/planner_server.cpp
)
ament_target_dependencies(planner_server
  rclcpp nav_msgs geometry_msgs pluginlib visualization_msgs
)
target_link_libraries(planner_server sp_global_planner_plugins)

install(TARGETS
  planner_server
  sp_global_planner_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
