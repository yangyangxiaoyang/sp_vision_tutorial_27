cmake_minimum_required(VERSION 3.8)
project(sp_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs    REQUIRED)
find_package(robot_msg   REQUIRED)
find_package(sp_msgs     REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  robot_msg
  sp_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
)

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

include_directories(include)

add_library(bt_engine_lib SHARED src/bt_engine.cpp)
ament_target_dependencies(bt_engine_lib ${dependencies})

add_library(blackboard_bridge_lib SHARED src/blackboard_bridge.cpp)
ament_target_dependencies(blackboard_bridge_lib ${dependencies})

add_library(decision_engine_lib SHARED src/decision_engine.cpp)
ament_target_dependencies(decision_engine_lib ${dependencies})
target_link_libraries(decision_engine_lib bt_engine_lib blackboard_bridge_lib)

add_library(decision_gui_lib SHARED
  src/decision_gui.cpp
  include/sp_decision/decision_gui.hpp
)
target_link_libraries(decision_gui_lib
  decision_engine_lib
  bt_engine_lib
  blackboard_bridge_lib
  Qt5::Core
  Qt5::Widgets
)
ament_target_dependencies(decision_gui_lib ${dependencies})

add_executable(sp_decision_node src/decision_main.cpp)
target_link_libraries(sp_decision_node
  decision_engine_lib
  decision_gui_lib
  bt_engine_lib
  blackboard_bridge_lib
  Qt5::Core
  Qt5::Widgets
)
ament_target_dependencies(sp_decision_node ${dependencies})

# ── click_nav.xml 所需 BT 插件 ──────────────────────────────────────────────
add_library(nav_to_pose_bt_node SHARED plugins/nav_to_pose.cpp)
ament_target_dependencies(nav_to_pose_bt_node ${dependencies})

add_library(condition_judge_bt_node SHARED plugins/condition_judge.cpp)
ament_target_dependencies(condition_judge_bt_node ${dependencies})
target_include_directories(condition_judge_bt_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_part>
)
target_compile_options(condition_judge_bt_node PRIVATE
  -Wno-unused-parameter -Wno-pedantic -Wno-extra
)

add_library(gimbal_control_bt_node SHARED plugins/gimbal_control.cpp)
ament_target_dependencies(gimbal_control_bt_node ${dependencies})

add_library(chassis_mode_bt_node SHARED plugins/chassis_mode.cpp)
ament_target_dependencies(chassis_mode_bt_node ${dependencies})

add_library(wait_duration_bt_node SHARED plugins/wait_duration.cpp)
ament_target_dependencies(wait_duration_bt_node ${dependencies})

add_library(set_double_bt_node SHARED plugins/set_double.cpp)
ament_target_dependencies(set_double_bt_node ${dependencies})

install(TARGETS
  bt_engine_lib
  blackboard_bridge_lib
  decision_engine_lib
  decision_gui_lib
  sp_decision_node
  nav_to_pose_bt_node
  condition_judge_bt_node
  gimbal_control_bt_node
  chassis_mode_bt_node
  wait_duration_bt_node
  set_double_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch decision_trees config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
