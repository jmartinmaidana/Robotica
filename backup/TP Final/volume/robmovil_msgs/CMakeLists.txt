cmake_minimum_required(VERSION 3.5)
project(robmovil_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Default to C++14 (not critical for pure msg packages, but standard in ROS 2)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# Find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)



#find_package(catkin REQUIRED COMPONENTS
#  geometry_msgs
#  message_generation
#)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  EncoderTicks.msg
#  MultiEncoderTicks.msg
#  Trajectory.msg
#  TrajectoryPoint.msg
#  Landmark.msg
#  LandmarkArray.msg
#)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderTicks.msg"
  "msg/MultiEncoderTicks.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Landmark.msg"
  "msg/LandmarkArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

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
#   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
#)
# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
