cmake_minimum_required(VERSION 3.5)
project(rrt_pioneer_planning)

# Enable C++11 support
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
#   rrt_planning
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
find_package(rrt_planning REQUIRED)
find_package(robmovil_planning REQUIRED)
find_package(angles REQUIRED)

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
#   CATKIN_DEPENDS roscpp robmovil_planning rrt_planning
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
add_executable(rrt_pioneer_planning src/rrt_pioneer_planning_node.cpp src/PioneerRRTPlanner.cpp)
#target_link_libraries(rrt_pioneer_planning ${catkin_LIBRARIES})
ament_target_dependencies(rrt_pioneer_planning rclcpp geometry_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs angles robmovil_planning rrt_planning)
target_link_libraries(rrt_pioneer_planning rrt_planning::rrt_planner)

install(TARGETS
  rrt_pioneer_planning
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

ament_package()