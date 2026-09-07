cmake_minimum_required(VERSION 3.8)
project(sp_map_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(esdf_map_publisher src/esdf_map_publisher.cpp)
ament_target_dependencies(esdf_map_publisher rclcpp nav_msgs std_msgs)
target_link_libraries(esdf_map_publisher
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS esdf_map_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
