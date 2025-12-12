cmake_minimum_required(VERSION 3.5)
project(rrt_planning)

# Enable C++11 support
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11")
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   tf
#   geometry_msgs
#   nav_msgs
#   robmovil_msgs
#   robmovil_planning
#   cmake_modules
# )
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robmovil_planning REQUIRED)

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
#   INCLUDE_DIRS include
#   LIBRARIES rrt_planner
#   CATKIN_DEPENDS roscpp robmovil_planning
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

## Declare a header-only (templated) C++ library
add_library(rrt_planner include/rrt_planning/RRTPlanner.h)
ament_target_dependencies(rrt_planner rclcpp geometry_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs robmovil_planning)
target_link_libraries(rrt_planner robmovil_planning::grid_planner)
set_target_properties(rrt_planner PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(rrt_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(rrt_planning::rrt_planner ALIAS rrt_planner)

install(TARGETS
  rrt_planner
  EXPORT export_grid_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_grid_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs robmovil_planning)

ament_package()