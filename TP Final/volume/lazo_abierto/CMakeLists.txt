cmake_minimum_required(VERSION 3.5)
project(lazo_abierto)

# Enable C++11 support
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
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
#  LIBRARIES trajectory_controller
#   CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
)

## Declare a C++ executable
add_library(trajectory_controller src/TrajectoryFollower.cpp)

add_executable(trajectory_follower src/trajectory_follower_node.cpp src/FeedForwardController.cpp)
add_executable(trajectory_generator src/trajectory_generator_node.cpp)

ament_target_dependencies(trajectory_controller rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(trajectory_follower rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)

add_library(lazo_abierto::trajectory_controller ALIAS trajectory_controller)

## Specify libraries to link a library or executable target against
#target_link_libraries(trajectory_follower trajectory_controller ${catkin_LIBRARIES})
#target_link_libraries(trajectory_generator ${catkin_LIBRARIES})

target_link_libraries(trajectory_follower trajectory_controller)

target_include_directories(trajectory_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Install targets
install(TARGETS
  trajectory_controller
  #trajectory_follower
  #trajectory_generator
  EXPORT export_trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  #trajectory_controller
  trajectory_follower
  trajectory_generator
  #EXPORT export_trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_targets(export_trajectory_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs robmovil_msgs nav_msgs tf2 tf2_geometry_msgs)

ament_package()