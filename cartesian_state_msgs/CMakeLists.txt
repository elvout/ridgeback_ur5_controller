cmake_minimum_required(VERSION 3.5)
project(cartesian_state_msgs)

## By default build with code optimization and no debug info
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   geometry_msgs
#   message_generation
# )

# add_message_files(
#   FILES
#   PoseTwist.msg
# )

# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
# )

# catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseTwist.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
