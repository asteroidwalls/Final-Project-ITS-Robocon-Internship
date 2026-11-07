cmake_minimum_required(VERSION 3.8)
project(robocon_2027_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tr_goto_pid src/tr_goto_pid_node.cpp)
target_include_directories(tr_goto_pid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tr_goto_pid PUBLIC cxx_std_17)
ament_target_dependencies(tr_goto_pid rclcpp geometry_msgs nav_msgs std_msgs)

install(TARGETS tr_goto_pid
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
