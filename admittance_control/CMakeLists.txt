cmake_minimum_required(VERSION 3.5)
project(admittance_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find ament macros and libraries
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_servo REQUIRED)

find_package(cartesian_state_msgs REQUIRED)

include_directories(include)

add_executable(admittance_controller_node
    src/admittance_controller_node.cpp
    src/AdmittanceController.cpp
)

ament_target_dependencies(admittance_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  moveit_servo
  cartesian_state_msgs
)
target_link_libraries(admittance_controller_node Eigen3::Eigen)

install(TARGETS admittance_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
