cmake_minimum_required(VERSION 3.5)
project(modelo_diferencial)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects

include_directories(include)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ executable
add_executable(pioneer_odometry_node src/pioneer_odometry_node.cpp src/pioneer_odometry.cpp)

## Specify libraries to link a library or executable target against

# Link libraries
ament_target_dependencies(pioneer_odometry_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  robmovil_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS pioneer_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament package macro
ament_package()