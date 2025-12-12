cmake_minimum_required(VERSION 3.5)
project(ros_intro)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  keyboard
#  geometry_msgs
#)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(keyboard_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robmovil_ekf
#   CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
#include_directories(
#  ${catkin_INCLUDE_DIRS}
#)

## Declare a C++ executable
#add_executable(keys_to_twist src/keys_to_twist_node.cpp src/keys_to_twist.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(keys_to_twist ${catkin_LIBRARIES})

# Add executable
add_executable(keys_to_twist
  src/keys_to_twist_node.cpp
  src/keys_to_twist.cpp
)

# Link against dependencies
ament_target_dependencies(keys_to_twist
  rclcpp
  geometry_msgs
  keyboard_msgs
)

# Install the node so ROS 2 can run it
install(TARGETS keys_to_twist
  DESTINATION lib/${PROJECT_NAME}
)

# This must always be at the end
ament_package()